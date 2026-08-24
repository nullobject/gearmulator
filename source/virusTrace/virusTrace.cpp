// Reference architectural trace of the Virus firmware running on the emulated
// DSP56300, for lockstep co-simulation against a gateware implementation.
//
// Phase 1 of the Pathogen FPGA port rests on this. A commercial firmware blob
// running on a from-scratch core cannot be debugged by listening for sound: the
// only error signal is silence, somewhere inside millions of instructions. A
// per-instruction reference trace turns that into "instruction 4,182,993, R3
// wrong by one".
//
// The trace records, before every instruction, the complete architectural state
// as it stood at that moment, plus every memory write the instruction performs.
// A gateware core replaying the same program must reproduce it exactly.
//
// Boot runs at full JIT speed. When the window opens the JIT is reconfigured to
// one instruction per block with no linking and every block is thrown away, so
// from that point each instruction is a separate compiled block bracketed by the
// trace hook - which is what makes the state coherent and the memory writes
// attributable to the instruction that made them.

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "virusTraceVersion.h"

#include "virusConsoleLib/consoleApp.h"
#include "dsp56kEmu/dsp.h"
#include "dsp56kEmu/memory.h"
#include "dsp56kEmu/memtrace.h"

// Bump whenever the record layout or the header keys change -- or, as for 3,
// when a version turns out to have been recording an incomplete set of writes:
// 48-bit L: stores went through a raw pointer that never reached the tracer, so
// a version 2 trace replays wrongly past any of them. The reader refuses a
// version it does not know rather than guessing at the fields.
constexpr int TRACE_FORMAT_VERSION = 3;

namespace
{
	FILE*    g_out = nullptr;
	uint64_t g_count = 0;
	uint64_t g_limit = 0;
	bool     g_done = false;

	// memory writes are collected as they happen and flushed with the next
	// instruction record, so each write is attributed to the instruction that
	// issued it rather than to the one that happens to follow
	struct Write { uint8_t area; uint32_t addr, value; };
	std::vector<Write> g_pendingWrites;

	void memSink(const uint8_t _area, const bool _write, const uint32_t _addr, const uint32_t _value, uint32_t)
	{
		if(_write && !g_done)
			g_pendingWrites.push_back({_area, _addr, _value});
	}

	const char* areaName(const uint8_t _area)
	{
		switch(_area)
		{
		case dsp56k::MemArea_X: return "X";
		case dsp56k::MemArea_Y: return "Y";
		default:                return "P";
		}
	}

	void flushWrites()
	{
		for(const auto& w : g_pendingWrites)
			fprintf(g_out, "W %s %06x %06x\n", areaName(w.area), w.addr, w.value);
		g_pendingWrites.clear();
	}

	// One line per instruction: every register a DSP56300 core has to get right.
	// Fixed field order, hex, no names - the header line documents it once.
	std::string g_memPrefix;

	// A trace is only replayable if the memory it started from is in it. Dumped
	// on the first record, which is the moment the window opens: any later
	// instruction can then be reached by replaying writes from here.
	void dumpMemory(dsp56k::DSP* _dsp)
	{
		FILE* f = fopen((g_memPrefix + ".mem").c_str(), "wb");
		if(!f)
			return;

		auto& mem = _dsp->memory();
		const dsp56k::EMemArea areas[3] = { dsp56k::MemArea_P, dsp56k::MemArea_X, dsp56k::MemArea_Y };
		const uint32_t sizes[3] = { mem.sizeP(), 0x20000, 0x20000 };	// X/Y alias P above the bridge

		// header: 3 words giving the length of each area, then packed 24-bit words
		for(uint32_t i = 0; i < 3; ++i)
			fwrite(&sizes[i], sizeof(uint32_t), 1, f);

		for(uint32_t i = 0; i < 3; ++i)
		{
			for(uint32_t a = 0; a < sizes[i]; ++a)
			{
				const auto v = mem.get(areas[i], a);
				const uint8_t b[3] = { static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v) };
				fwrite(b, 1, 3, f);
			}
		}
		fclose(f);
		fprintf(stderr, "wrote memory snapshot to %s.mem\n", g_memPrefix.c_str());
	}

	void instSink(dsp56k::DSP* _dsp, const uint32_t _pc)
	{
		if(g_done)
			return;

		if(g_count == 0 && !g_memPrefix.empty())
			dumpMemory(_dsp);

		// writes issued by the *previous* instruction are only known now
		flushWrites();

		if(g_count >= g_limit)
		{
			g_done = true;
			return;
		}
		++g_count;

		const auto& r = _dsp->readRegs();
		const auto sc = static_cast<uint32_t>(r.sc.var) & 0x1f;

		fprintf(g_out, "I %06x %06x %06x %014" PRIx64 " %014" PRIx64 " %012" PRIx64 " %012" PRIx64,
			_pc,
			r.sr.var & 0xffffff, r.omr.var & 0xffffff,
			static_cast<uint64_t>(r.a.var) & 0xffffffffffffffull,
			static_cast<uint64_t>(r.b.var) & 0xffffffffffffffull,
			static_cast<uint64_t>(r.x.var) & 0xffffffffffffull,
			static_cast<uint64_t>(r.y.var) & 0xffffffffffffull);

		for(const auto& v : r.r) fprintf(g_out, " %06x", v.var & 0xffffff);
		for(const auto& v : r.n) fprintf(g_out, " %06x", v.var & 0xffffff);
		for(const auto& v : r.m) fprintf(g_out, " %06x", v.var & 0xffffff);

		// the whole 16-entry stack is not worth a line each time; sp, sc and the
		// current top are enough to catch a push/pop or a return going wrong
		const auto top = sc ? static_cast<uint64_t>(r.ss[sc & 0xf].var) : 0ull;

		fprintf(g_out, " %06x %06x %02x %012" PRIx64 " %06x %06x %06x %06x\n",
			r.sp.var & 0xffffff, r.la.var & 0xffffff, sc, top & 0xffffffffffffull,
			r.lc.var & 0xffffff, r.vba.var & 0xffffff, r.ep.var & 0xffffff, r.sz.var & 0xffffff);
	}
}

int main(int argc, char* argv[])
{
	if(argc < 2)
	{
		fprintf(stderr,
			"usage: virusTrace <rom> [out] [instructions] [startFrame] [voices]\n"
			"  out           trace file, default /tmp/virus.trace\n"
			"  instructions  how many to record, default 100000\n"
			"  startFrame    audio frame to open the window at, default 4096\n"
			"                (note-on is at 2048, so the default is just after it)\n"
			"  voices        note-ons to hold during the window, default 1\n");
		return 1;
	}

	const std::string rom   = argv[1];
	const std::string out   = argc > 2 ? argv[2] : "/tmp/virus.trace";
	g_limit                 = argc > 3 ? strtoull(argv[3], nullptr, 10) : 100000;
	const uint32_t startFrm = argc > 4 ? static_cast<uint32_t>(atoi(argv[4])) : 4096;
	const int voices        = argc > 5 ? atoi(argv[5]) : 1;

	ConsoleApp app(rom, virusLib::DeviceModel::ABC);
	if(!app.isValid())
	{
		fprintf(stderr, "ROM not valid\n");
		return 1;
	}

	app.loadSingle(0, 0);

	std::vector<uint8_t> notes;
	for(int i = 0; i < voices; ++i)
		notes.push_back(static_cast<uint8_t>(36 + i));
	app.setNotes(notes);

	g_out = fopen(out.c_str(), "w");
	if(!g_out)
	{
		fprintf(stderr, "cannot write %s\n", out.c_str());
		return 1;
	}

	// One "key: value" per line rather than key=value on one line, so a ROM path
	// containing spaces cannot break the parse. The reader requires the version
	// and refuses anything it does not understand: a trace is the only thing the
	// core is validated against, so a silent misparse is the worst failure here.
	fprintf(g_out, "# virusTrace %d\n", TRACE_FORMAT_VERSION);
	fprintf(g_out, "# gearmulator: %s\n", VIRUSTRACE_GEARMULATOR_SHA);
	fprintf(g_out, "# dsp56300: %s\n", VIRUSTRACE_DSP56300_SHA);
	fprintf(g_out, "# rom: %s\n", app.getRom().getFilename().c_str());
	fprintf(g_out, "# model: %d\n", static_cast<int>(app.getRom().getModel()));
	fprintf(g_out, "# samplerate: %d\n", app.getRom().getSamplerate());
	fprintf(g_out, "# start-frame: %u\n", startFrm);
	fprintf(g_out, "# voices: %d\n", voices);
	fprintf(g_out, "# memory: %s.mem\n", out.c_str());
	fprintf(g_out, "# fields: pc sr omr a56 b56 x48 y48 r0..r7 n0..n7 m0..m7 sp la sc sstop lc vba ep sz\n");
	fprintf(g_out, "#\n");
	fprintf(g_out, "# I <fields>  - state as it stood BEFORE the instruction at pc executes\n");
	fprintf(g_out, "# W area addr value  - a memory write by the preceding I\n");

	g_memPrefix = out;
	dsp56k::instTraceSetSink(&instSink);
	dsp56k::memTraceSetSink(&memSink);
	app.enableInstTrace();
	app.setMemTraceWindow(0, 0xffffffff, startFrm, 0xfffffff0);

	// the window has to stay open long enough to retire the requested count; one
	// frame is ~2300 instructions, and lockstep mode is far slower than a normal
	// render, so ask for a generous but bounded number of frames
	const auto frames = startFrm + static_cast<uint32_t>(g_limit / 512) + 64;

	fprintf(stderr, "tracing %" PRIu64 " instructions from frame %u...\n", g_limit, startFrm);
	app.run(out + ".wav", frames, 64, false, false);

	flushWrites();
	fclose(g_out);

	fprintf(stderr, "wrote %" PRIu64 " instructions to %s\n", g_count, out.c_str());

	// the render length is derived from the instruction count by a rule of thumb,
	// and a firmware that idles retires far fewer instructions per frame than one
	// that does not - so say when the window closed early rather than silently
	// handing back a shorter trace than was asked for
	if(g_count < g_limit)
		fprintf(stderr, "WARNING: asked for %" PRIu64 " but the window closed after %" PRIu64 ".\n"
		                "         Raise startFrame's render budget by asking for more instructions,\n"
		                "         or trace a busier part of the run.\n", g_limit, g_count);

	return g_count ? 0 : 1;
}
