// Empirical Access Virus TI reverb characterisation.
//
// Reverb Send = 127 ("Effect") makes the output 100% wet, so a short broadband
// noise burst yields the reverb's impulse response directly. From the IR we
// derive: RT60 (Schroeder backward integration), frequency-dependent decay
// (low vs high band -> damping behaviour), and echo spacing (autocorrelation
// -> delay-line / comb structure). Reverb params live at preset byte 256+index.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "virusConsoleLib/consoleApp.h"
#include "dsp56kEmu/memtrace.h"

using TPreset = virusLib::ROMFile::TPreset;

namespace
{
	constexpr int A = 0;      // page-A byte base
	constexpr int B = 128;    // page-B byte base
	constexpr int RV = 256;   // reverb page (110) byte base

	void P(TPreset& p, int base, int idx, uint8_t v) { p[base + idx] = v; }

	struct RevCfg { int type, time, damping, predelay, color, feedback, mode; };

	// Sustained tonal patch (filtered saw -> strong fundamental line) into a
	// 100% wet reverb. Used to test time-variance: an LTI reverb (FDN/Schroeder)
	// keeps the line razor-sharp; a modulated reverb (Dattorro-style, LFO on
	// tank allpasses) smears it / adds sidebands.
	void buildToneReverbPatch(TPreset& p, const RevCfg& c)
	{
		P(p, A, 34, 0); P(p, A, 37, 0); P(p, A, 38, 0);  // sub/noise/ring off
		P(p, A, 36, 100);                 // Osc main volume
		P(p, A, 40, 40);                  // Cutoff low -> filtered saw ~ near sine
		P(p, A, 42, 0);                   // Resonance 0
		P(p, A, 44, 0); P(p, A, 46, 64);  // no env, keyfollow 0
		P(p, A, 51, 0);                   // LP
		P(p, A, 59, 0); P(p, A, 60, 127); P(p, A, 61, 127); P(p, A, 63, 40); // sustained
		P(p, A, 7, 127); P(p, A, 91, 110);
		P(p, A, 67, 0); P(p, A, 79, 0); P(p, B, 7, 0);
		P(p, A, 77, 64); P(p, A, 88, 64); P(p, A, 89, 64);
		for (int i = 64; i <= 111; ++i) P(p, B, i, 0);
		P(p, A, 105, 0); P(p, A, 112, 0); P(p, B, 85, 0); P(p, B, 101, 0); P(p, B, 97, 0);
		P(p, RV, 1, uint8_t(c.mode)); P(p, RV, 2, 127); P(p, RV, 3, uint8_t(c.type));
		P(p, RV, 4, uint8_t(c.time)); P(p, RV, 5, uint8_t(c.damping)); P(p, RV, 6, uint8_t(c.color));
		P(p, RV, 7, uint8_t(c.predelay)); P(p, RV, 8, 0); P(p, RV, 9, uint8_t(c.feedback));
	}

	void buildReverbPatch(TPreset& p, const RevCfg& c)
	{
		// short broadband noise burst as the excitation
		P(p, A, 34, 0);   // Sub vol
		P(p, A, 36, 0);   // Osc main vol
		P(p, A, 37, 127); // Noise vol
		P(p, A, 38, 0);   // Ring vol
		P(p, A, 40, 127); // Cutoff wide open
		P(p, A, 42, 0);   // Resonance
		P(p, A, 44, 0);   // Filter1 env amt
		P(p, A, 46, 64);  // keyfollow 0
		P(p, A, 51, 0);   // LP
		P(p, A, 59, 0);   // Amp Atk 0
		P(p, A, 60, 24);  // Amp Dec (short)
		P(p, A, 61, 0);   // Amp Sus 0  -> percussive click
		P(p, A, 63, 20);  // Amp Rel
		P(p, A, 7, 127);  // Channel vol
		P(p, A, 91, 110); // Patch vol

		// freeze LFOs / mods that could colour things
		P(p, A, 67, 0); P(p, A, 79, 0); P(p, B, 7, 0);
		P(p, A, 77, 64); P(p, A, 88, 64); P(p, A, 89, 64);
		for (int i = 64; i <= 111; ++i) P(p, B, i, 0); // mod matrix off

		// kill every other effect
		P(p, A, 105, 0);  // Chorus Mix
		P(p, A, 112, 0);  // Delay Mode = Off
		P(p, B, 85, 0);   // Phaser Mix
		P(p, B, 101, 0);  // Distortion Intensity
		P(p, B, 97, 0);   // Bass/Character Intensity

		// reverb
		P(p, RV, 1, uint8_t(c.mode));      // Reverb Mode (1=Reverb)
		P(p, RV, 2, 127);                  // Reverb Send = 127 ("Effect" = 100% wet)
		P(p, RV, 3, uint8_t(c.type));      // Reverb Type
		P(p, RV, 4, uint8_t(c.time));      // Reverb Time
		P(p, RV, 5, uint8_t(c.damping));   // Reverb Damping
		P(p, RV, 6, uint8_t(c.color));     // Reverb Color
		P(p, RV, 7, uint8_t(c.predelay));  // Reverb Predelay
		P(p, RV, 8, 0);                    // Reverb Clock (no sync)
		P(p, RV, 9, uint8_t(c.feedback));  // Reverb Feedback
	}

	// ---- 24-bit PCM stereo WAV reader; returns left channel (mono) ----
	bool readWavLeft(const std::string& f, std::vector<double>& out, uint32_t& sr)
	{
		std::ifstream s(f, std::ios::binary);
		if (!s) return false;
		std::vector<uint8_t> d((std::istreambuf_iterator<char>(s)), std::istreambuf_iterator<char>());
		if (d.size() < 44) return false;
		auto r32 = [&](size_t o){ return d[o]|(d[o+1]<<8)|(d[o+2]<<16)|(uint32_t(d[o+3])<<24); };
		auto r16 = [&](size_t o){ return uint16_t(d[o]|(d[o+1]<<8)); };
		uint16_t ch=2, bits=24; sr=44100; size_t pos=12, off=0, len=0;
		while (pos+8 <= d.size()) {
			uint32_t id=r32(pos), sz=r32(pos+4);
			if (id==0x20746d66){ ch=r16(pos+10); sr=r32(pos+12); bits=r16(pos+22); }
			else if (id==0x61746164){ off=pos+8; len=sz; break; }
			pos += 8+sz+(sz&1);
		}
		if (!off || bits!=24) return false;
		const size_t frame=3*ch, n=len/frame;
		out.resize(n);
		for (size_t i=0;i<n;++i){ size_t o=off+i*frame; int32_t v=d[o]|(d[o+1]<<8)|(d[o+2]<<16); if(v&0x800000)v|=0xff000000; out[i]=double(v)/8388608.0; }
		return true;
	}

	// biquad (RBJ) band helpers
	struct BQ { double b0,b1,b2,a1,a2,z1=0,z2=0;
		double run(double x){ double y=b0*x+z1; z1=b1*x-a1*y+z2; z2=b2*x-a2*y; return y; } };
	BQ lowpass(double fc,double sr){ double w=2*M_PI*fc/sr,c=cos(w),s=sin(w),al=s/std::sqrt(2.0);
		double a0=1+al; BQ q; q.b0=(1-c)/2/a0; q.b1=(1-c)/a0; q.b2=q.b0; q.a1=-2*c/a0; q.a2=(1-al)/a0; return q; }
	BQ highpass(double fc,double sr){ double w=2*M_PI*fc/sr,c=cos(w),s=sin(w),al=s/std::sqrt(2.0);
		double a0=1+al; BQ q; q.b0=(1+c)/2/a0; q.b1=-(1+c)/a0; q.b2=q.b0; q.a1=-2*c/a0; q.a2=(1-al)/a0; return q; }

	std::vector<double> filt(const std::vector<double>& x, BQ q){ std::vector<double> y(x.size()); for(size_t i=0;i<x.size();++i)y[i]=q.run(x[i]); return y; }

	size_t onset(const std::vector<double>& x){
		double pk=0; for(double v:x) pk=std::max(pk,std::fabs(v));
		double th=pk*0.05; for(size_t i=0;i<x.size();++i) if(std::fabs(x[i])>th) return i; return 0; }

	// Floor-aware RT60. Uses the instantaneous energy-decay envelope (RMS in
	// 20ms hops, in dB). Estimates the noise floor from the quietest hop, fits
	// a line from -5 dB down to whichever comes first: -30 dB or (floor+4 dB),
	// requiring >=12 dB of usable span. Returns seconds, or 99 if the tail
	// never decays that far within the render (effectively non-decaying).
	double rt60(const std::vector<double>& x, size_t start, double sr){
		size_t n=x.size(); if(start+2000>=n) return -1;
		const size_t hop=(size_t)(0.02*sr);
		std::vector<double> db; std::vector<double> t;
		double ref=-1, floor=1e9;
		for(size_t i=start;i+hop<n;i+=hop){ double s=0; for(size_t j=0;j<hop;++j){double v=x[i+j];s+=v*v;} double rms=std::sqrt(s/hop);
			if(ref<0)ref=rms; double d=20*std::log10(std::max(rms,1e-12)/ref); db.push_back(d); t.push_back((i-start)/sr); floor=std::min(floor,d); }
		if(db.size()<8) return -1;
		double lo = std::max(-30.0, floor+4.0);
		if(-5.0 - lo < 12.0) return 99; // not enough dynamic range above floor -> non-decaying
		long i5=-1,iL=-1;
		for(size_t i=0;i<db.size();++i){ if(i5<0&&db[i]<=-5) i5=(long)i; if(db[i]<=lo){iL=(long)i;break;} }
		if(i5<0||iL<0||iL<=i5) return 99;
		double slope=(db[iL]-db[i5])/(t[iL]-t[i5]); // dB/s (neg)
		if(slope>=0) return 99;
		return -60.0/slope;
	}

	// dominant autocorrelation lag (ms) in an early tail window -> comb/delay spacing
	double autocorrLagMs(const std::vector<double>& x, size_t start, double sr){
		size_t w=(size_t)(0.30*sr); if(start+w>=x.size()) w=x.size()-start-1;
		size_t minLag=(size_t)(0.001*sr), maxLag=(size_t)(0.060*sr);
		double best=-1; size_t bl=0;
		for(size_t lag=minLag; lag<maxLag; ++lag){ double s=0; for(size_t i=0;i<w-lag;++i) s+=x[start+i]*x[start+i+lag];
			if(s>best){best=s;bl=lag;} }
		return 1000.0*bl/sr; }

	// echo density: local maxima above 0.2*localpeak per second in first 120ms of tail
	double echoDensity(const std::vector<double>& x, size_t start, double sr){
		size_t w=(size_t)(0.12*sr); if(start+w>=x.size()) w=x.size()-start-1;
		double pk=0; for(size_t i=0;i<w;++i) pk=std::max(pk,std::fabs(x[start+i]));
		double th=pk*0.2; int cnt=0;
		for(size_t i=start+1;i<start+w-1;++i){ double a=std::fabs(x[i]); if(a>th&&a>=std::fabs(x[i-1])&&a>std::fabs(x[i+1])) cnt++; }
		return cnt/0.12; }
}

int main(int argc,char* argv[]){
	std::string rom = argc>1?argv[1]:"";
	std::string exp = argc>2?argv[2]:"types";

	// memory-dump mode: apply a reverb config, settle in SILENCE (render stops
	// before the note-on fires at ~2048 frames, so delay buffers stay zero and
	// only the reverb *configuration* is written), then dump X/Y memory.
	//   virusReverbProbe <rom> mem <type> <time> <damping> <prefix>
	if(exp=="mem"){
		RevCfg c{ argc>3?atoi(argv[3]):3, argc>4?atoi(argv[4]):100,
		          argc>5?atoi(argv[5]):0, 0, 64, 0, 1 };
		std::string prefix = argc>6?argv[6]:"/tmp/vmem";
		ConsoleApp app(rom, virusLib::DeviceModel::TI2);
		if(!app.isValid()){ fprintf(stderr,"ROM not valid\n"); return 1; }
		app.loadSingle(0,0);
		buildReverbPatch(app.editPreset(), c);
		app.setMemDumpPrefix(prefix);
		app.run("/tmp/vmem.wav", 2000 /*frames, < note-on*/, 64, false, false);
		fprintf(stderr,"dumped %s_X.txt / _Y.txt (type=%d time=%d damp=%d)\n",
			prefix.c_str(), c.type, c.time, c.damping);
		return 0;
	}

	// modulation test: sustained tone into 100% wet reverb; measure the tail's
	// spectral line shape. Sharp line -> time-invariant (FDN/Schroeder). Broadened
	// / sidebands -> modulated (Dattorro-style tank).  virusReverbProbe <rom> modtest
	if(exp=="modtest"){
		const char* tn[4]={"Ambience","SmallRoom","LargeRoom","Hall"};
		for(int tp=0; tp<4; ++tp){
			ConsoleApp app(rom, virusLib::DeviceModel::TI2);
			if(!app.isValid()){ fprintf(stderr,"ROM not valid\n"); return 1; }
			app.loadSingle(0,0);
			buildToneReverbPatch(app.editPreset(), {tp, 96, 0, 0, 64, 0, 1});
			char wav[128]; snprintf(wav,sizeof(wav),"/tmp/vmod_%d.wav",tp);
			app.run(wav, 44100*4, 64, false, false);
			std::vector<double> x; uint32_t sr=44100;
			if(!readWavLeft(wav,x,sr)||x.empty()){ fprintf(stderr,"read fail\n"); continue; }
			// steady-state tail window 2.0..3.8s
			size_t s0=(size_t)(2.0*sr), s1=std::min(x.size(),(size_t)(3.8*sr)); size_t n=s1-s0;
			auto g=[&](double f){ double w=2*M_PI*f/sr,c=2*cos(w),a0=0,a1=0,a2=0;
				for(size_t i=0;i<n;++i){a0=x[s0+i]+c*a1-a2;a2=a1;a1=a0;} return std::sqrt(a1*a1+a2*a2-c*a1*a2); };
			// coarse find fundamental 50..2000Hz
			double pf=0,pm=-1; for(double f=50;f<2000;f*=1.002){double m=g(f); if(m>pm){pm=m;pf=f;}}
			// fine line shape +/-25 Hz at 0.25 Hz
			double peak=g(pf); double halfBW=0; int side=0; double sideMax=0,sideF=0;
			for(double d=0.25; d<=25; d+=0.25){ double ml=g(pf-d),mr=g(pf+d); double mm=std::max(ml,mr);
				if(halfBW==0 && mm < peak*0.5) halfBW=d;
				if(d>2 && mm>sideMax){sideMax=mm;sideF=d;} }
			double sidedb=20*std::log10(std::max(sideMax,1e-12)/peak);
			printf("type=%-9s  fund=%.1fHz  -6dB_halfwidth=%.2fHz  worst_sideband=%.1fdB @±%.1fHz\n",
				tn[tp], pf, halfBW, sidedb, sideF);
			fflush(stdout);
		}
		return 0;
	}

	// topology trace: fire an impulse into the reverb and log every DSP write to
	// the delay-buffer region during a window after it. Address motion -> delay
	// lengths; value propagation -> mixing structure (matrix=FDN vs serial=Moorer).
	//   virusReverbProbe <rom> trace <type> [sil]
	if(exp=="trace"){
		int type = argc>3?atoi(argv[3]):3;
		bool silent = (argc>4 && std::string(argv[4])=="sil");
		ConsoleApp app(rom, virusLib::DeviceModel::TI2);
		if(!app.isValid()){ fprintf(stderr,"ROM not valid\n"); return 1; }
		app.loadSingle(0,0);
		RevCfg c{type, 90, 0, 0, 64, 0, 1};
		buildReverbPatch(app.editPreset(), c);
		if(silent) P(app.editPreset(), A, 37, 0); // no noise -> pure silence (map buffers)
		app.enableMemWriteTracing();
		// note-on ~2048 frames; trace the delay-buffer region right after it
		app.setMemTraceWindow(0x44000, 0x4e000, silent?6000:2100, silent?11000:5200);
		app.run("/tmp/vtrace.wav", 12000, 64, false, false);

		const auto& tr = dsp56k::memTraceData();
		char fn[128]; snprintf(fn,sizeof(fn),"/tmp/vtrace_%d%s.txt",type,silent?"_sil":"");
		FILE* f=fopen(fn,"w");
		for(const auto& e : tr) fprintf(f,"%u %06x %06x\n", e.area, e.addr, e.value);
		fclose(f);
		fprintf(stderr,"wrote %zu trace entries to %s\n", tr.size(), fn);
		return 0;
	}

	// experiment matrices: {type,time,damping,predelay,color,feedback,mode}
	std::vector<std::pair<std::string,RevCfg>> runs;
	const char* typeNames[4]={"Ambience","SmallRoom","LargeRoom","Hall"};
	auto add=[&](const std::string& n,RevCfg c){ runs.push_back({n,c}); };

	if(exp=="types"){
		for(int t=0;t<4;++t) add(typeNames[t], {t, 100, 0, 0, 64, 0, 1});
	} else if(exp=="time"){
		for(int tm : {0,16,32,48,64,80,96,112,127}) add("Hall_T"+std::to_string(tm), {3, tm, 0, 0, 64, 0, 1});
	} else if(exp=="damping"){
		for(int dp : {0,32,64,96,127}) add("Hall_D"+std::to_string(dp), {3, 100, dp, 0, 64, 0, 1});
	} else if(exp=="predelay"){
		for(int pd : {0,4,10,20,40,92}) add("Hall_P"+std::to_string(pd), {3, 64, 0, pd, 64, 0, 1});
	} else if(exp=="modes"){
		for(int md : {1,2,3}) add("Mode"+std::to_string(md), {3, 100, 0, 0, 64, 64, md});
	} else if(exp=="profile"){
		for(int tm : {32,64,96,127}) add("Hall_T"+std::to_string(tm), {3, tm, 0, 0, 64, 0, 1});
	} else { fprintf(stderr,"unknown experiment %s\n",exp.c_str()); return 1; }

	const bool profile = (exp=="profile");

	printf("# Virus TI reverb probe: experiment=%s (100%% wet, 44.1kHz)\n", exp.c_str());
	printf("# %-14s  RT_low_s  RT_mid_s  RT_high_s  echoDens/s  acLag_ms\n","name");
	fflush(stdout);

	for(auto& r : runs){
		ConsoleApp app(rom, virusLib::DeviceModel::TI2);
		if(!app.isValid()){ fprintf(stderr,"ROM not valid\n"); return 1; }
		app.loadSingle(0,0);
		buildReverbPatch(app.editPreset(), r.second);
		char wav[256]; snprintf(wav,sizeof(wav),"/tmp/vrev_%s.wav", r.first.c_str());
		app.run(wav, 44100*5 /*5s*/, 64, false, false);

		std::vector<double> x; uint32_t sr=44100;
		if(!readWavLeft(wav,x,sr)||x.empty()){ fprintf(stderr,"read fail %s\n",wav); continue; }
		x = filt(x, highpass(30,sr)); // remove DC / subsonic drift
		size_t on=onset(x);
		size_t tailStart=on+(size_t)(0.03*sr); // skip the direct burst

		auto band=[&](double lo,double hi){ auto y=filt(x,highpass(lo,sr)); y=filt(y,lowpass(hi,sr)); return y; };
		double rtL=rt60(band(60,400),tailStart,sr);
		double rtM=rt60(band(600,2000),tailStart,sr);
		double rtH=rt60(filt(x,highpass(5000,sr)),tailStart,sr);
		double ed=echoDensity(x,on,sr);
		double ac=autocorrLagMs(x,tailStart,sr);
		printf("%-16s  %8.2f  %8.2f  %9.2f  %9.0f  %7.2f\n",
			r.first.c_str(), rtL, rtM, rtH, ed, ac);
		fflush(stdout);

		if(profile){
			// RMS in dB per 250ms window relative to first window
			const size_t win=(size_t)(0.25*sr); double ref=-1;
			printf("   decay dB/250ms:");
			for(size_t w=0; w<20 && tailStart+(w+1)*win<x.size(); ++w){
				double s=0; for(size_t i=0;i<win;++i){ double v=x[tailStart+w*win+i]; s+=v*v; }
				double rms=std::sqrt(s/win); if(ref<0)ref=rms;
				printf(" %.0f", 20*std::log10(std::max(rms,1e-9)/ref));
			}
			printf("\n"); fflush(stdout);
		}
	}
	return 0;
}
