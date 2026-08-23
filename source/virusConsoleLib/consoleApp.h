#pragma once
#include <functional>
#include <string>
#include <vector>

#include "virusLib/romfile.h"
#include "virusLib/microcontroller.h"
#include "virusLib/demoplayback.h"
#include "virusLib/dspSingle.h"

class ConsoleApp
{
public:
	ConsoleApp(const std::string& _romFile, virusLib::DeviceModel _tiModel);
	~ConsoleApp();

	bool isValid() const;

	void loadSingle(int b, int p);
	bool loadSingle(const std::string& _preset);

	bool loadDemo(const std::string& _filename);
	bool loadInternalDemo();

	std::string getSingleName() const;
	std::string getSingleNameAsFilename() const;

	static void waitReturn();

	void run(const std::string& _audioOutputFilename, uint32_t _maxSampleCount = 0, uint32_t _blockSize = 64, bool _createDebugger = false, bool _dumpAssembler = false);

	const virusLib::ROMFile& getRom() const { return m_rom; }

	// analysis helper: mutate the currently loaded single before run()
	virusLib::ROMFile::TPreset& editPreset() { return m_preset; }

	// analysis helper: dump DSP X/Y memory to <prefix>_X.txt / _Y.txt at end of run()
	void setMemDumpPrefix(std::string _p) { m_memDumpPrefix = std::move(_p); }

	// analysis helper: route all DSP memory writes through C++ so they can be traced
	void enableMemWriteTracing();
	// analysis helper: route memory accesses through C++ from before boot, so the
	// firmware's own load of P/X/Y over HDI08 is traced too. Unlike
	// enableMemWriteTracing() this costs boot speed, which whole-run profiling
	// needs to pay: blocks compiled before the config change keep the fast path.
	void enableFullMemTracing() { m_fullMemTrace = true; }

	// analysis helper: count instruction fetch. Must be armed before any JIT block
	// is compiled, so run() enables it ahead of boot. The counters are zeroed again
	// when the trace window opens, which keeps firmware upload out of the numbers.
	void enableFetchProfiling() { m_fetchProfile = true; }

	// analysis helper: DSP cycles and instructions retired inside the trace window,
	// so reconstructed fetch counts can be sanity-checked against them
	uint64_t getWindowCycles() const { return m_windowCycles; }
	uint64_t getWindowInstructions() const { return m_windowInstructions; }

	// analysis helper: notes played at the note-on point instead of the single
	// middle C, so voice count can be swept
	// an empty list is meaningful: it means play nothing, which is the baseline
	// that separates work from idle
	void setNotes(std::vector<uint8_t> _notes) { m_notes = std::move(_notes); m_notesSet = true; }

	// analysis helper: inspect DSP memory right after boot / right before teardown
	using MemCallback = std::function<void(dsp56k::Memory&)>;
	void setPreBootCallback(MemCallback _c) { m_preBoot = std::move(_c); }
	void setPostBootCallback(MemCallback _c) { m_postBoot = std::move(_c); }
	void setPostRunCallback(MemCallback _c) { m_postRun = std::move(_c); }

	// analysis helper: record memory writes in [lo,hi) during sample window [start,end)
	void setMemTraceWindow(uint32_t _lo, uint32_t _hi, uint32_t _start, uint32_t _end)
	{ m_traceLo = _lo; m_traceHi = _hi; m_traceStart = _start; m_traceEnd = _end; }

private:

	void bootDSP(bool _createDebugger) const;
	dsp56k::IPeripherals& getYPeripherals() const;
	void audioCallback(uint32_t _audioCallbackCount);
	void destroy();

	const std::string m_romName;
	virusLib::ROMFile m_rom;
	std::unique_ptr<virusLib::DspSingle> m_dsp1;
	virusLib::DspSingle* m_dsp2 = nullptr;
	std::unique_ptr<virusLib::Microcontroller> m_uc;
	std::unique_ptr<virusLib::DemoPlayback> m_demo;

	virusLib::Microcontroller::TPreset m_preset;
	std::string m_memDumpPrefix;

	bool m_memTraceEnabled = false;
	bool m_fullMemTrace = false;
	bool m_fetchProfile = false;
	uint64_t m_windowCycles = 0, m_windowInstructions = 0;
	std::vector<uint8_t> m_notes;
	bool m_notesSet = false;
	MemCallback m_preBoot, m_postBoot, m_postRun;
	uint32_t m_traceLo = 0, m_traceHi = 0, m_traceStart = 0, m_traceEnd = 0;
};
