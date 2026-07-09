#pragma once
#include <string>

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

	uint32_t m_traceLo = 0, m_traceHi = 0, m_traceStart = 0, m_traceEnd = 0;
};
