// Access Virus DSP memory-footprint profiler.
//
// Phase 0 of the Pathogen FPGA port asks a single question: how much of the
// DSP56300 address space does the firmware actually populate? The answer sets
// the BRAM budget, and no clock speed or speed grade can rescue it if the
// firmware needs more zero-wait RAM than the fabric holds.
//
// Two measurements, cheap first:
//
//  scan  - poison every word with 0xabcabcab before boot. The firmware never
//          writes that value, so any word differing from it after a run was
//          written. That is the *exact* write footprint, at no runtime cost.
//          Split into what boot loaded and what the audio code wrote after.
//          (dsp56k::Memory has g_useInitPattern = false, so without the poison
//          untouched memory is indistinguishable from a word written as zero.)
//          Pass poison=0 to render without it; the two .wav files must match
//          bit for bit, otherwise the poison is perturbing the emulation and
//          the footprint numbers cannot be trusted.
//  trace - additionally routes every DSP access through C++ so reads are seen
//          too. Words read but never written can live in ROM; only the written
//          set has to be RAM. Also profiles instruction fetch: the JIT compiles
//          a block once and runs it many times, so fetch traffic is reconstructed
//          as (times each block ran) x (that block's length in P words). Both
//          measure the window from note-on onwards, so firmware upload is out.
//
// Geometry (virusLib): ABC allocates 0x40000 words with external memory from
// 0x020000. X and Y exist only below that bridge point (0x20000 words each);
// at and above it, X, Y and P all alias one shared external bank.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "virusConsoleLib/consoleApp.h"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/memtrace.h"

namespace
{
	// Address-stamped sentinel. A flat pattern is not enough: DSP words are 24
	// bits, so a stale copy of the sentinel written from anywhere else would read
	// back as "untouched". Stamping the address means a word only looks untouched
	// if it still holds its own stamp.
	uint32_t stamp(const uint8_t _area, const uint32_t _addr)
	{
		return ((_addr * 0x9e3779u) ^ (0xa5a5a5u * (_area + 1u)) ^ 0x5c3d2bu) & 0xffffffu;
	}
	constexpr uint32_t g_bridge      = 0x020000;    // external memory start
	constexpr uint32_t g_maxWords    = 0x040000;
	constexpr uint32_t g_pageWords   = 256;
	constexpr uint32_t g_pages       = g_maxWords / g_pageWords;

	// physical banks, after X/Y >= bridge are folded onto P
	enum Bank { X_INT, Y_INT, P_INT, EXT, BANK_COUNT };
	const char* g_bankName[BANK_COUNT] = { "X internal", "Y internal", "P internal", "external shared" };
	const uint32_t g_bankWords[BANK_COUNT] = { g_bridge, g_bridge, g_bridge, g_maxWords - g_bridge };

	Bank physBank(const uint8_t _area, const uint32_t _addr)
	{
		if(_addr >= g_bridge)
			return EXT;
		switch(_area)
		{
		case dsp56k::MemArea_X: return X_INT;
		case dsp56k::MemArea_Y: return Y_INT;
		default:                return P_INT;
		}
	}

	uint32_t physOffset(const uint32_t _addr) { return _addr >= g_bridge ? _addr - g_bridge : _addr; }

	// trace accumulators: one flag byte per physical word, plus per-page counters
	constexpr uint8_t F_READ = 1, F_WRITE = 2;
	std::vector<uint8_t>  g_flags[BANK_COUNT];
	uint64_t              g_pageReads[BANK_COUNT][g_pages]  = {};
	uint64_t              g_pageWrites[BANK_COUNT][g_pages] = {};
	uint64_t              g_bankReads[BANK_COUNT] = {}, g_bankWrites[BANK_COUNT] = {};

	// Instruction fetch is invisible to the JIT call path, but every traced access
	// carries the PC of the code that made it. Histogramming that says where the
	// audio code itself lives - internal P, or the external bank behind a wait state.
	std::vector<uint8_t>  g_pcSeen;
	uint64_t              g_pcAccInt = 0, g_pcAccExt = 0;
	uint64_t              g_totalReads = 0, g_totalWrites = 0;

	void traceSink(const uint8_t _area, const bool _write, const uint32_t _addr, uint32_t, const uint32_t _pc)
	{
		if(_pc < g_maxWords)
		{
			g_pcSeen[_pc] = 1;
			(_pc >= g_bridge ? g_pcAccExt : g_pcAccInt)++;
		}

		if(_addr >= g_maxWords)
			return;
		const auto b = physBank(_area, _addr);
		const auto o = physOffset(_addr);
		if(o >= g_bankWords[b])
			return;
		g_flags[b][o] |= _write ? F_WRITE : F_READ;
		(_write ? g_pageWrites : g_pageReads)[b][o / g_pageWords]++;
		(_write ? g_bankWrites : g_bankReads)[b]++;
		(_write ? g_totalWrites : g_totalReads)++;
	}

	// snapshot of "which words differ from the fill pattern", per physical bank
	using Snapshot = std::vector<std::vector<uint8_t>>;

	bool g_poisoned = true;

	Snapshot snapshot(dsp56k::Memory& _mem)
	{
		Snapshot s(BANK_COUNT);
		for(int b = 0; b < BANK_COUNT; ++b)
			s[b].assign(g_bankWords[b], 0);

		const dsp56k::EMemArea areas[3] = { dsp56k::MemArea_X, dsp56k::MemArea_Y, dsp56k::MemArea_P };
		for(const auto area : areas)
		{
			// below the bridge each area is its own bank; at and above it, only
			// P is read (X and Y alias the same words and would double count)
			const uint32_t hi = (area == dsp56k::MemArea_P) ? g_maxWords : g_bridge;
			for(uint32_t a = 0; a < hi; ++a)
			{
				const auto v = _mem.get(area, a);
				if(v == (g_poisoned ? stamp(static_cast<uint8_t>(area), a) : 0u))
					continue;
				const auto b = physBank(static_cast<uint8_t>(area), a);
				s[b][physOffset(a)] = 1;
			}
		}
		return s;
	}

	void poison(dsp56k::Memory& _mem)
	{
		for(uint32_t a = 0; a < g_bridge; ++a)
		{
			_mem.set(dsp56k::MemArea_X, a, stamp(dsp56k::MemArea_X, a));
			_mem.set(dsp56k::MemArea_Y, a, stamp(dsp56k::MemArea_Y, a));
		}
		for(uint32_t a = 0; a < g_maxWords; ++a)
			_mem.set(dsp56k::MemArea_P, a, stamp(dsp56k::MemArea_P, a));
	}

	uint32_t count(const std::vector<uint8_t>& _v, const uint8_t _mask)
	{
		uint32_t n = 0;
		for(const auto v : _v) if(v & _mask) ++n;
		return n;
	}

	uint32_t highWater(const std::vector<uint8_t>& _v, const uint8_t _mask)
	{
		for(size_t i = _v.size(); i--;) if(_v[i] & _mask) return static_cast<uint32_t>(i);
		return 0;
	}

	struct Blk { uint32_t pc, words, instrs; uint64_t count; };

	// 24-bit words expressed as ECP5 block RAM. A DP16KD holds 18432 bits; this
	// is the perfect-packing floor, so the real cost is higher.
	double bramsFor(const uint64_t _words) { return static_cast<double>(_words) * 24.0 / 18432.0; }
}

int main(int argc, char* argv[])
{
	if(argc < 2)
	{
		fprintf(stderr,
			"usage: virusMemProfile <rom> [mode] [seconds] [voices] [prefix]\n"
			"  mode    scan (default, exact write footprint) | trace (adds read footprint, slow)\n"
			"  seconds render length after note-on, default 2\n"
			"  voices  simultaneous note-ons, default 1\n"
			"  prefix  output path prefix, default /tmp/vprof\n"
			"  poison  1 (default) fills memory with a sentinel first; 0 renders without it\n"
			"          so the two .wav files can be compared to prove the fill is inert\n");
		return 1;
	}

	const std::string rom    = argv[1];
	const std::string mode   = argc > 2 ? argv[2] : "scan";
	const double      secs   = argc > 3 ? atof(argv[3]) : 2.0;
	const int         voices = argc > 4 ? atoi(argv[4]) : 1;
	const std::string prefix = argc > 5 ? argv[5] : "/tmp/vprof";
	const bool        doPoison = argc > 6 ? atoi(argv[6]) != 0 : true;
	g_poisoned = doPoison;

	const bool tracing = (mode == "trace");

	ConsoleApp app(rom, virusLib::DeviceModel::ABC);
	if(!app.isValid())
	{
		fprintf(stderr, "ROM not valid\n");
		return 1;
	}

	printf("# model=%d samplerate=%d\n", static_cast<int>(app.getRom().getModel()), app.getRom().getSamplerate());

	app.loadSingle(0, 0);

	// distinct pitches: a repeated note-on retriggers the same voice rather than
	// adding one, so the notes must not collide
	// voices=0 renders silence: the baseline that separates real work from idle
	std::vector<uint8_t> notes;
	for(int i = 0; i < voices; ++i)
		notes.push_back(static_cast<uint8_t>(36 + i));
	app.setNotes(notes);

	// measure from note-on, so the firmware upload is not counted as steady state
	constexpr uint32_t g_noteOnFrame = 2048;

	if(tracing)
	{
		for(int b = 0; b < BANK_COUNT; ++b)
			g_flags[b].assign(g_bankWords[b], 0);
		g_pcSeen.assign(g_maxWords, 0);
		dsp56k::memTraceSetSink(&traceSink);
		app.enableFullMemTracing();
		app.enableFetchProfiling();
		app.setMemTraceWindow(0, 0xffffffff, g_noteOnFrame, 0xfffffff0);
	}

	Snapshot bootSnap, runSnap;
	if(doPoison)
		app.setPreBootCallback([&](dsp56k::Memory& _m) { poison(_m); });
	app.setPostBootCallback([&](dsp56k::Memory& _m) { bootSnap = snapshot(_m); });
	app.setPostRunCallback ([&](dsp56k::Memory& _m) { runSnap  = snapshot(_m); });

	// note-on lands at sample 2048; render for the requested time past it
	const auto sr = static_cast<uint32_t>(app.getRom().getSamplerate());
	const auto frames = 2048 + static_cast<uint32_t>(secs * sr);

	printf("# rendering %u frames (%.2fs after note-on) with %d voice(s), mode=%s poison=%d\n",
		frames, secs, voices, mode.c_str(), doPoison ? 1 : 0);
	fflush(stdout);

	app.run(prefix + ".wav", frames, 64, false, false);

	if(runSnap.empty())
	{
		fprintf(stderr, "no post-run snapshot\n");
		return 1;
	}

	// ---- report -----------------------------------------------------------
	if(!doPoison)
		printf("\n# NOTE: poison disabled - counts below are non-zero words, not written words\n");
	printf("\n# write footprint (words differing from the fill pattern)\n");
	printf("# %-16s %8s %10s %10s %10s   %s\n", "bank", "words", "at boot", "after run", "added", "high water");
	uint64_t totalWritten = 0;
	for(int b = 0; b < BANK_COUNT; ++b)
	{
		const auto atBoot = count(bootSnap[b], 1);
		const auto after  = count(runSnap[b], 1);
		totalWritten += after;
		printf("  %-16s %8u %10u %10u %10d   0x%05x\n", g_bankName[b], g_bankWords[b],
			atBoot, after, static_cast<int>(after) - static_cast<int>(atBoot),
			highWater(runSnap[b], 1) + (b == EXT ? g_bridge : 0));
	}
	printf("  %-16s %8u %10s %10llu\n", "TOTAL", 3 * g_bridge + (g_maxWords - g_bridge), "",
		static_cast<unsigned long long>(totalWritten));
	printf("\n  written words = %llu  ->  %.1f Kbit at 24 bits, %.1f DP16KD perfectly packed\n",
		static_cast<unsigned long long>(totalWritten), totalWritten * 24.0 / 1024.0, bramsFor(totalWritten));

	if(tracing)
	{
		printf("\n# access footprint (%llu reads, %llu writes traced over %u frames)\n",
			static_cast<unsigned long long>(g_totalReads), static_cast<unsigned long long>(g_totalWrites), frames);
		printf("# NOTE: the JIT call path sees data accesses only. Instruction fetch is\n"
		       "#       invisible, as are the loader's own writes during firmware upload,\n"
		       "#       so P and boot-loaded regions are undercounted here - see the scan\n"
		       "#       table above for those.\n");
		printf("# %-16s %10s %10s %10s %10s %12s %12s\n",
			"bank", "touched", "written", "read-only", "untouched", "rd/frame", "wr/frame");
		uint64_t needRam = 0, needRom = 0;
		for(int b = 0; b < BANK_COUNT; ++b)
		{
			const auto touched  = count(g_flags[b], F_READ | F_WRITE);
			const auto written  = count(g_flags[b], F_WRITE);
			const auto readOnly = touched - written;
			needRam += written;
			needRom += readOnly;
			printf("  %-16s %10u %10u %10u %10u %12.1f %12.1f\n", g_bankName[b], touched, written, readOnly,
				g_bankWords[b] - touched,
				static_cast<double>(g_bankReads[b]) / frames, static_cast<double>(g_bankWrites[b]) / frames);
		}
		printf("\n  RAM (written)   = %llu words -> %.1f DP16KD perfectly packed\n",
			static_cast<unsigned long long>(needRam), bramsFor(needRam));
		printf("  ROM (read only) = %llu words -> %.1f DP16KD perfectly packed\n",
			static_cast<unsigned long long>(needRom), bramsFor(needRom));

		// where the code making those accesses lives
		uint32_t pcInt = 0, pcExt = 0, pcLo = g_maxWords, pcHi = 0;
		for(uint32_t a = 0; a < g_maxWords; ++a)
		{
			if(!g_pcSeen[a])
				continue;
			(a >= g_bridge ? pcExt : pcInt)++;
			if(a < pcLo) pcLo = a;
			if(a > pcHi) pcHi = a;
		}
		const auto pcAcc = g_pcAccInt + g_pcAccExt;
		printf("\n# code making those accesses (block-granular PC, so a floor on the code footprint)\n");
		printf("  distinct PCs   internal %u, external %u   range 0x%05x..0x%05x\n", pcInt, pcExt, pcLo, pcHi);
		if(pcAcc)
			printf("  accesses issued from internal P %.1f%%, from external %.1f%%\n",
				100.0 * static_cast<double>(g_pcAccInt) / static_cast<double>(pcAcc),
				100.0 * static_cast<double>(g_pcAccExt) / static_cast<double>(pcAcc));

		// ---- instruction fetch ---------------------------------------------
		const auto window = frames > g_noteOnFrame ? frames - g_noteOnFrame : frames;
		uint64_t fetchInt = 0, fetchExt = 0;
		std::vector<Blk> blocks;
		std::vector<uint8_t> codeSeen(g_maxWords, 0);
		if(dsp56k::fetchProfileActive())
		{
			const auto* counts = dsp56k::fetchProfileCounts();
			const auto* sizes  = dsp56k::fetchProfileSizes();
			const auto* instrs = dsp56k::fetchProfileInstrs();
			for(uint32_t pc = 0; pc < dsp56k::fetchProfileSize() && pc < g_maxWords; ++pc)
			{
				if(!counts[pc])
					continue;
				const auto w = sizes[pc];
				blocks.push_back({pc, w, instrs[pc], counts[pc]});
				(pc >= g_bridge ? fetchExt : fetchInt) += counts[pc] * w;
				for(uint32_t i = 0; i < w && pc + i < g_maxWords; ++i)
					codeSeen[pc + i] = 1;
			}
		}
		uint32_t codeInt = 0, codeExt = 0;
		for(uint32_t a = 0; a < g_maxWords; ++a)
			if(codeSeen[a]) (a >= g_bridge ? codeExt : codeInt)++;

		uint64_t blockInstrs = 0;
		for(const auto& b : blocks)
			blockInstrs += b.count * b.instrs;

		const auto fetchTotal = fetchInt + fetchExt;
		if(fetchTotal)
		{
			printf("\n# instruction fetch (block executions x block length, %u frames after note-on)\n", window);
			printf("  DSP retired %.1f instructions and %.1f cycles per frame (both include idle)\n",
				static_cast<double>(app.getWindowInstructions()) / window,
				static_cast<double>(app.getWindowCycles()) / window);
			// The same block entries also carry the pacing counter, so summing them
			// must reproduce it. A shortfall is idle: DSP::op_Wait fast-forwards
			// both m_instructions and m_cycles to the next peripheral event, so
			// neither counter is a measure of work - a WAIT-heavy firmware inflates
			// both while fetching nothing. This percentage is the real duty cycle.
			printf("  block entries account for %.1f instructions/frame - a duty cycle of %.1f%%.\n"
			       "  The remainder is WAIT: op_Wait fast-forwards both counters to the next\n"
			       "  peripheral event, so retired instructions and cycles both include idle.\n",
				static_cast<double>(blockInstrs) / window,
				100.0 * static_cast<double>(blockInstrs) / static_cast<double>(app.getWindowInstructions()));
			if(const auto bad = dsp56k::fetchProfileUnencodable())
				printf("# WARNING: %u blocks could not be counted (counter out of reach of the JIT's\n"
				       "#          base register), so the totals below are an undercount\n", bad);
			printf("  fetch words  internal %12llu  external %12llu  (%.1f%% external)\n",
				static_cast<unsigned long long>(fetchInt), static_cast<unsigned long long>(fetchExt),
				100.0 * static_cast<double>(fetchExt) / static_cast<double>(fetchTotal));
			printf("  per frame    internal %12.1f  external %12.1f\n",
				static_cast<double>(fetchInt) / window, static_cast<double>(fetchExt) / window);
			printf("  rate         external %.2f M words/s at %u Hz\n",
				static_cast<double>(fetchExt) / window * sr / 1e6, sr);
			printf("  code executed  %u words internal, %u words external, %zu distinct blocks\n",
				codeInt, codeExt, blocks.size());

			// how much instruction memory would remove how much external fetch
			std::sort(blocks.begin(), blocks.end(),
				[](const Blk& _a, const Blk& _b) { return _a.count * _a.words > _b.count * _b.words; });
			printf("\n  # hot-code concentration: cache the top blocks and this much fetch stays on chip\n");
			printf("  # %10s %12s %12s\n", "code words", "% of fetch", "% of ext");
			uint64_t cum = 0, cumExt = 0;
			uint32_t cumWords = 0;
			size_t next = 1;
			for(size_t i = 0; i < blocks.size(); ++i)
			{
				const auto t = blocks[i].count * blocks[i].words;
				cum += t;
				cumWords += blocks[i].words;
				if(blocks[i].pc >= g_bridge)
					cumExt += t;
				if(i + 1 == next || i + 1 == blocks.size())
				{
					printf("  %12u %11.2f%% %11.2f%%\n", cumWords,
						100.0 * static_cast<double>(cum) / static_cast<double>(fetchTotal),
						fetchExt ? 100.0 * static_cast<double>(cumExt) / static_cast<double>(fetchExt) : 0.0);
					next *= 2;
				}
			}

			if(FILE* bf = fopen((prefix + "_blocks.txt").c_str(), "w"))
			{
				fprintf(bf, "# pc words executions fetch_words\n");
				for(const auto& b : blocks)
					fprintf(bf, "0x%05x %u %llu %llu\n", b.pc, b.words,
						static_cast<unsigned long long>(b.count),
						static_cast<unsigned long long>(b.count * b.words));
				fclose(bf);
				printf("\n  wrote block map to %s_blocks.txt\n", prefix.c_str());
			}
		}

		// what an FPGA port has to serve from off-chip memory, per audio frame
		const auto extAcc = g_bankReads[EXT] + g_bankWrites[EXT];
		const auto intAcc = g_totalReads + g_totalWrites - extAcc;
		printf("\n# total external-bank traffic an FPGA port must serve\n");
		printf("  internal-bank data accesses %12.1f / frame\n", static_cast<double>(intAcc) / window);
		printf("  external-bank data accesses %12.1f / frame\n", static_cast<double>(extAcc) / window);
		printf("  external instruction fetch  %12.1f / frame\n", static_cast<double>(fetchExt) / window);
		printf("  external TOTAL              %12.1f / frame = %.2f M/s at %u Hz\n",
			static_cast<double>(extAcc + fetchExt) / window,
			static_cast<double>(extAcc + fetchExt) / window * sr / 1e6, sr);
	}

	// ---- page map ---------------------------------------------------------
	const auto pageFile = prefix + "_pages.txt";
	if(FILE* f = fopen(pageFile.c_str(), "w"))
	{
		fprintf(f, "# bank page_addr written_words boot_words reads writes\n");
		for(int b = 0; b < BANK_COUNT; ++b)
		{
			const auto pages = g_bankWords[b] / g_pageWords;
			for(uint32_t p = 0; p < pages; ++p)
			{
				uint32_t w = 0, bw = 0;
				for(uint32_t i = 0; i < g_pageWords; ++i)
				{
					w  += runSnap[b][p * g_pageWords + i];
					bw += bootSnap.empty() ? 0 : bootSnap[b][p * g_pageWords + i];
				}
				const auto r  = tracing ? g_pageReads[b][p]  : 0;
				const auto wr = tracing ? g_pageWrites[b][p] : 0;
				if(!w && !bw && !r && !wr)
					continue;
				fprintf(f, "%s 0x%05x %u %u %llu %llu\n", g_bankName[b],
					p * g_pageWords + (b == EXT ? g_bridge : 0), w, bw,
					static_cast<unsigned long long>(r), static_cast<unsigned long long>(wr));
			}
		}
		fclose(f);
		printf("\nwrote page map to %s\n", pageFile.c_str());
	}

	return 0;
}
