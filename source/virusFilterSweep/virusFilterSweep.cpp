// Empirical Access Virus (TI) filter cutoff -> frequency mapping.
//
// For each MIDI Cutoff value (0..127) we build a resonant low-pass patch with:
//   - all oscillators / sub / ring silenced, a little noise for excitation
//   - Filter1 = LowPass, Resonance = 127, Env Amt = 0, Keyfollow = center (0)
//   - all LFO/mod-matrix routing to cutoff neutralised
// then hold a note and render. At max resonance the spectral peak sits at the
// filter cutoff frequency, so we Goertzel-scan the steady-state output and
// report the peak frequency. That IS the cutoff-in-Hz for that value.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

#include "virusConsoleLib/consoleApp.h"

using TPreset = virusLib::ROMFile::TPreset;

namespace
{
	// page-A parameter index == byte offset in the single dump.
	// page-B parameter index == 128 + byte offset.
	constexpr int A = 0;
	constexpr int B = 128;

	void setParam(TPreset& _p, int _base, int _index, uint8_t _value)
	{
		_p[_base + _index] = _value;
	}

	void buildResonantLowpass(TPreset& _p, uint8_t _cutoff)
	{
		// --- kill all sound sources except a touch of noise for excitation ---
		setParam(_p, A, 34, 0);    // Suboscillator Volume
		setParam(_p, A, 36, 0);    // Osc Mainvolume
		setParam(_p, A, 37, 90);   // Noise Volume (excitation)
		setParam(_p, A, 38, 0);    // Ringmodulator Volume
		setParam(_p, A, 39, 0);    // Noise Color (0 = neutral)

		// --- filter 1 = resonant low-pass at the requested cutoff ---
		setParam(_p, A, 40, _cutoff); // Cutoff
		setParam(_p, A, 42, 127);     // Filter1 Resonance (max -> peak at cutoff)
		setParam(_p, A, 44, 0);       // Filter1 Env Amt (no env on cutoff)
		setParam(_p, A, 46, 64);      // Filter1 Keyfollow (center = 0 = no tracking)
		setParam(_p, A, 48, 0);       // Filter Balance (fully Filter1)
		setParam(_p, A, 51, 0);       // Filter1 Mode = LowPass
		setParam(_p, A, 53, 0);       // Filter Routing = Serial 4
		setParam(_p, A, 43, 0);       // Filter2 Resonance
		setParam(_p, A, 45, 0);       // Filter2 Env Amt

		// --- envelopes: instant, full sustain so the tone is stationary ---
		setParam(_p, A, 54, 0);       // Filter Env Attack
		setParam(_p, A, 55, 127);     // Filter Env Decay
		setParam(_p, A, 56, 127);     // Filter Env Sustain
		setParam(_p, A, 59, 0);       // Amp Env Attack
		setParam(_p, A, 60, 127);     // Amp Env Decay
		setParam(_p, A, 61, 127);     // Amp Env Sustain
		setParam(_p, A, 63, 64);      // Amp Env Release

		// --- levels ---
		setParam(_p, A, 7, 127);      // Channel Volume
		setParam(_p, A, 91, 110);     // Patch Volume

		// --- neutralise LFO -> cutoff/reso direct amounts (center = 0) ---
		setParam(_p, A, 77, 64);      // Reso  Lfo1 Amount
		setParam(_p, A, 88, 64);      // Cutoff1 Lfo2 Amount
		setParam(_p, A, 89, 64);      // Cutoff2 Lfo2 Amount
		setParam(_p, A, 67, 0);       // Lfo1 Rate = 0 (frozen)
		setParam(_p, A, 79, 0);       // Lfo2 Rate = 0 (frozen)
		setParam(_p, B, 7, 0);        // Lfo3 Rate = 0
		setParam(_p, B, 12, 0);       // Osc Lfo3 Amount

		// --- wipe the whole mod-matrix region (page-B idx 64..111) ->
		//     every Assign source becomes 0 (Off), all amounts 0 ---
		for (int idx = 64; idx <= 111; ++idx)
			setParam(_p, B, idx, 0);
	}

	// ---- minimal 24-bit PCM stereo WAV reader; returns left channel as doubles ----
	bool readWavLeft(const std::string& _file, std::vector<double>& _out, uint32_t& _sr)
	{
		std::ifstream f(_file, std::ios::binary);
		if (!f) return false;
		std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		if (d.size() < 44) return false;

		auto rd32 = [&](size_t o){ return d[o] | (d[o+1]<<8) | (d[o+2]<<16) | (uint32_t(d[o+3])<<24); };
		auto rd16 = [&](size_t o){ return uint16_t(d[o] | (d[o+1]<<8)); };

		uint16_t channels = 2, bits = 24;
		_sr = 44100;
		size_t pos = 12, dataOff = 0, dataLen = 0;
		while (pos + 8 <= d.size())
		{
			const uint32_t id = rd32(pos);
			const uint32_t sz = rd32(pos + 4);
			if (id == 0x20746d66) // "fmt "
			{
				channels = rd16(pos + 10);
				_sr = rd32(pos + 12);
				bits = rd16(pos + 22);
			}
			else if (id == 0x61746164) // "data"
			{
				dataOff = pos + 8;
				dataLen = sz;
				break;
			}
			pos += 8 + sz + (sz & 1);
		}
		if (!dataOff || bits != 24) return false;

		const size_t bytesPerSample = 3;
		const size_t frame = bytesPerSample * channels;
		const size_t frames = dataLen / frame;
		_out.resize(frames);
		for (size_t i = 0; i < frames; ++i)
		{
			const size_t o = dataOff + i * frame;
			int32_t v = d[o] | (d[o+1] << 8) | (d[o+2] << 16);
			if (v & 0x800000) v |= 0xff000000; // sign extend
			_out[i] = double(v) / 8388608.0;
		}
		return true;
	}

	// Goertzel magnitude at frequency f over the given window
	double goertzel(const std::vector<double>& x, size_t start, size_t n, double f, double sr)
	{
		const double w = 2.0 * M_PI * f / sr;
		const double c = 2.0 * std::cos(w);
		double s0 = 0, s1 = 0, s2 = 0;
		for (size_t i = 0; i < n; ++i)
		{
			s0 = x[start + i] + c * s1 - s2;
			s2 = s1;
			s1 = s0;
		}
		return std::sqrt(s1*s1 + s2*s2 - c*s1*s2);
	}

	// find dominant frequency in a steady-state window via a log-spaced Goertzel scan
	double dominantFreq(const std::vector<double>& x, double sr, double& _rms)
	{
		// steady-state window: 1.0s .. 1.9s (skip attack / note trigger)
		size_t start = size_t(1.0 * sr);
		size_t end   = size_t(1.9 * sr);
		if (end > x.size()) end = x.size();
		if (start + 4096 > end) { start = 0; end = x.size(); }
		const size_t n = end - start;

		double sum = 0;
		for (size_t i = 0; i < n; ++i) sum += x[start+i]*x[start+i];
		_rms = std::sqrt(sum / double(n ? n : 1));

		const double fmin = 8.0, fmax = sr * 0.49;
		const int bins = 3000;
		double bestF = 0, bestMag = -1;
		for (int b = 0; b < bins; ++b)
		{
			const double f = fmin * std::pow(fmax / fmin, double(b) / (bins - 1));
			const double m = goertzel(x, start, n, f, sr);
			if (m > bestMag) { bestMag = m; bestF = f; }
		}
		return bestF;
	}
}

int main(int _argc, char* _argv[])
{
	const std::string romArg = _argc > 1 ? _argv[1] : std::string();
	int step = _argc > 2 ? atoi(_argv[2]) : 4;
	if (step < 1) step = 1;
	const int lo = _argc > 3 ? atoi(_argv[3]) : 0;
	const int hi = _argc > 4 ? atoi(_argv[4]) : 127;

	std::vector<int> cutoffs;
	for (int c = lo; c <= hi; c += step) cutoffs.push_back(c);
	if (cutoffs.empty() || cutoffs.back() != hi) cutoffs.push_back(hi);

	// dump mode: boot once and write the P-memory disassembly, then exit
	if (_argc > 2 && std::string(_argv[2]) == "dump")
	{
		ConsoleApp app(romArg, virusLib::DeviceModel::TI2);
		if (!app.isValid()) { fprintf(stderr, "ROM not valid\n"); return 1; }
		app.loadSingle(0, 0);
		app.run("/tmp/vdump.wav", 44100 /*1s*/, 64, false, true /*dumpAssembler*/);
		fprintf(stderr, "assembly dumped (see <rom>_P.asm / _X.txt / _Y.txt next to rom)\n");
		return 0;
	}

	printf("# Access Virus TI filter cutoff sweep\n");
	printf("# cutoff  freqHz     rms\n");
	fflush(stdout);

	std::vector<std::pair<int,double>> results;

	for (const int cutoff : cutoffs)
	{
		ConsoleApp app(romArg, virusLib::DeviceModel::TI2);
		if (!app.isValid())
		{
			fprintf(stderr, "ROM not valid / not found (arg='%s')\n", romArg.c_str());
			return 1;
		}
		app.loadSingle(0, 0); // valid base single (structure/version), then override
		buildResonantLowpass(app.editPreset(), uint8_t(cutoff));

		char wav[256];
		snprintf(wav, sizeof(wav), "/tmp/vsweep_%03d.wav", cutoff);
		app.run(wav, 44100 * 2 /*2s*/, 64, false, false);

		std::vector<double> left; uint32_t sr = 44100;
		if (!readWavLeft(wav, left, sr) || left.empty())
		{
			fprintf(stderr, "failed to read %s\n", wav);
			continue;
		}
		double rms = 0;
		const double f = dominantFreq(left, double(sr), rms);
		results.emplace_back(cutoff, f);
		printf("%7d  %9.2f  %8.5f\n", cutoff, f, rms);
		fflush(stdout);
	}

	printf("\n# CSV\ncutoff,freq_hz\n");
	for (auto& r : results) printf("%d,%.2f\n", r.first, r.second);
	return 0;
}
