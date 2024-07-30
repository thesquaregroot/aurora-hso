/** Harmonic Shift Operator
 *
 * Inspired by the New System Instruments Harmonic Shift Oscillator, this extends
 * the ideas of harmonic stride and level into a spectral audio processor.
 *
 * Controls:
 *	- Warp: Base frequency coarse adjustment.  CV is exponential FM (ranging from -6 to +6 octaves).
 *	- Time: Base frequency fine adjusment (10% range of coarse).  CV is linear FM (20% of frequency).
 *	- Blur: Resonance, self-oscillation after 75% of knob range.
 *	- Reflect: Haromonic stride (distance between harmonics), ranging from 0 to 5.
 *	- Atomosphere: Harmonic level (relative level of each harmonic), ranging from 0 to 1.
 *	- Mix: Blend between dry signal and processed signal.
 *	- Reverse: Change sign of stride (generate/collect subharmonics instead).
 *	- Freeze: Filter cutoff mode. This treats the stride as 0 for input processing, effectively becoming
 *				a brick-wall high-pass (or, if reverse is activated, low-pass) filter with resonance character
 *				determined by stride/level controls.
 */
#include <cstdlib> // for srand & rand
#include <string>
#include <cmath> // for isfinite
#include "aurora.h"
#include "daisysp.h"
#include "fft/shy_fft.h"

using namespace std;
using namespace stmlib;
using namespace daisy;
using namespace aurora;
using namespace daisysp;

#define SAMPLE_RATE 48000
#define AUDIO_BLOCK_SIZE 256
#define DFT_SIZE 4096
#define OSCILLATOR_COUNT 4

#define FREQUENCY_MIN 20
#define FREQUENCY_MAX 16000
#define HARMONIC_MIN 1
#define HARMONIC_DROP_LOW 5
#define HARMONIC_DROP_HIGH 20000
#define HARMONIC_MAX 24000

#define CONFIG_FILE_NAME "HSO.txt"
#define CONFIG_REVERSE "INVERT_REVERSE"
#define CONFIG_FREEZE "INVERT_FREEZE"

Hardware hw;
bool isUsbConnected = false;
bool isUsbConfigured = false;
bool wasConfigLoadAttempted = false;
bool isConfigChanged = false;

#define LOG_ENABLED 0

#if LOG_ENABLED
#define LOG_FILE_NAME "HSO.log"
#define LOG_ITERATIONS 1000
#define SEC_TO_USEC 1000000
uint32_t logIteration = 0;
uint32_t setupTotal = 0;
uint32_t resonanceTotal = 0;
uint32_t processTotal = 0;
uint32_t outputTotal = 0;
uint32_t callbackTotal = 0;

TimerHandle createTimer(TimerHandle::Config::Peripheral peripheral) {
	TimerHandle::Config config;
	config.periph = peripheral;
	TimerHandle timer;
	timer.Init(config);
	return timer;
}

FIL logFile;
FRESULT logFileResult;
char logFilePath[100];
char logFileBuffer[256];
bool isLogWritePending = false;
bool isFirstWrite = true;

bool writeLog() {
	logFileResult = f_open(&logFile, logFilePath, (FA_OPEN_APPEND | FA_WRITE));
	if (logFileResult) {
		hw.SetLed(LED_1, logFileResult == FR_DISK_ERR, logFileResult == FR_INT_ERR, logFileResult == FR_NOT_READY);
		hw.SetLed(LED_2, 1.0, 1.0, 1.0); // white
		hw.SetLed(LED_3, logFileResult == FR_NO_FILE, logFileResult == FR_NO_PATH, logFileResult == FR_INVALID_NAME);
		hw.SetLed(LED_4, 1.0, 1.0, 1.0); // white
		hw.SetLed(LED_5, logFileResult == FR_DENIED, logFileResult == FR_EXIST, logFileResult == FR_INVALID_OBJECT);
		hw.WriteLeds();
		return false;
	}

	if (isFirstWrite) {
		f_puts("New boot.\n", &logFile);
		isFirstWrite = false;
	}

	f_puts(logFileBuffer, &logFile);
	f_close(&logFile);
	return true;
}
#endif

const double DFT_SIZE_RECIP = 1.0 / DFT_SIZE;
const int BIN_COUNT = DFT_SIZE / 2;
const double BIN_AMPLITUDE_RECIP = 2.0 / DFT_SIZE;
const double FREQUENCY_TO_BIN = 2.0 / SAMPLE_RATE * BIN_COUNT;
const double BIN_WIDTH = (SAMPLE_RATE / 2.0) / BIN_COUNT;
const float STRIDE_EPSILON = 0.01; // arbitrary small value
const float LEVEL_EPSILON = 0.01; // -40dB

typedef float dft_t;

template<typename T, size_t _capacity>
class _RingBuffer {
private:
	T _data[_capacity];
	size_t _start = 0;
	size_t _size = 0;

public:
	_RingBuffer() {}

	T operator[](int index) const {
		return _data[(_start + index) % _capacity];
	}

	void put(const T value) {
		if (_size < _capacity) {
			_data[(_start + _size) % _capacity] = value;
			_size += 1;
		}
		else {
			_data[_start] = value;
			_start = (_start + 1) % _capacity;
		}
	}

	size_t size() const {
		return _size;
	}

	size_t capacity() const {
		return _capacity;
	}

	T* ptr() {
		return _data;
	}
};

typedef _RingBuffer<dft_t, DFT_SIZE> Buffer;

inline float lerp(float a, float b, float t) {
	return a + t * (b - a);
}

inline float inverse_lerp(float c, float a, float b) {
	return (c - a) / (b - a);
}

Oscillator leftOscillators[OSCILLATOR_COUNT];
Oscillator rightOscillators[OSCILLATOR_COUNT];
double leftResonance[AUDIO_BLOCK_SIZE];
double rightResonance[AUDIO_BLOCK_SIZE];

Switch reverseButton;
bool isReverseActive = false;
bool isReverseInverted = false; // flips gate interpretation, changed by user input
Switch freezeButton;
bool isFreezeActive = false;
bool isFreezeInverted = false; // flips gate interpretation, changed by user input

DTCMRAM Buffer leftSignal; // input signal for left channel
DTCMRAM Buffer rightSignal; // input signal for right channel

// variables for tracking averages (for LEDs)
Buffer leftOuts; // output signal for left channel
Buffer rightOuts; // output signal for right channel
DTCMRAM double leftInTotal = 0;
DTCMRAM double rightInTotal = 0;
DTCMRAM double leftOutTotal = 0;
DTCMRAM double rightOutTotal = 0;

DTCMRAM Math<dft_t> math;
DTCMRAM ShyFFT<dft_t, DFT_SIZE> dft;
DTCMRAM dft_t window[DFT_SIZE];

DTCMRAM dft_t signalBuffer[2*DFT_SIZE];
dft_t* leftSignalBuffer = signalBuffer;
dft_t* rightSignalBuffer = signalBuffer + DFT_SIZE;

DTCMRAM dft_t spectrumBuffer[2*DFT_SIZE];
dft_t* leftSpectrum = spectrumBuffer;
dft_t* leftSpectrumReal = leftSpectrum;
dft_t* leftSpectrumImag = leftSpectrum + BIN_COUNT;
dft_t* rightSpectrum = spectrumBuffer + DFT_SIZE;
dft_t* rightSpectrumReal = rightSpectrum;
dft_t* rightSpectrumImag = rightSpectrum + BIN_COUNT;

dft_t indexPhase(size_t index) { return index / (dft_t)(DFT_SIZE-1); }
dft_t hann(dft_t phase) { return 0.5 * (1 - cos(2 * math.pi() * phase)); }

void processSignals(float baseFrequency, float strideFactor, float levelFactor, float resonance) {
	for (int i = 0; i < DFT_SIZE; i++) {
		leftSignalBuffer[i] = leftSignal[i] * window[i];
		rightSignalBuffer[i] = rightSignal[i] * window[i];
	}
	dft.Direct(leftSignalBuffer, leftSpectrum);
	dft.Direct(rightSignalBuffer, rightSpectrum);

	double baseLevel = (1.0 + resonance) * BIN_AMPLITUDE_RECIP; // account for resonance boost and DFT scaling
	size_t cutoffBin = baseFrequency * FREQUENCY_TO_BIN;
	for (size_t bin = 0; bin < BIN_COUNT; bin++) {
		double binLevel = baseLevel;

		// clear if below the cutoff
		//bool shouldClear = (bin < cutoffBin); // isReverseActive ? (bin > cutoffBin) : (bin < cutoffBin);
		//binLevel *= shouldClear ? 0 : 1;

		// determine appropriate level scale for bin
		/*bool isFilter = isFreezeActive;
		double binFrequency = (bin + 0.5) * BIN_WIDTH;
		size_t nearestPartialIndex = fastlog2f(binFrequency / baseFrequency) / fastlog2f(1 + strideFactor);
		size_t partialBin = baseFrequency * pow(1 + strideFactor, nearestPartialIndex) * FREQUENCY_TO_BIN;
		bool hasDesiredPartial = partialBin == bin;
		//binLevel *= hasDesiredPartial * (isFilter ? 1.0 : pow(levelFactor, nearestPartialIndex));
		binLevel *= hasDesiredPartial;*/

		leftSpectrumReal[bin] *= binLevel;
		leftSpectrumImag[bin] *= binLevel;
		rightSpectrumReal[bin] *= binLevel;
		rightSpectrumImag[bin] *= binLevel;
	}
	// perform inverse transform
	dft.Inverse(leftSpectrum, signalBuffer);
	dft.Inverse(rightSpectrum, signalBuffer+DFT_SIZE);
}

dft_t limit(dft_t value, dft_t previousValue) {
	if (!isfinite(value)) {
		return previousValue;
	}
	return SoftClip(value);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
#if LOG_ENABLED
	TimerHandle timer = createTimer(TimerHandle::Config::Peripheral::TIM_2);
	uint32_t usecFreq = timer.GetFreq() / SEC_TO_USEC;
	timer.Start();
	uint32_t callbackStart = timer.GetTick();
	uint32_t setupStart = timer.GetTick();
#endif
	hw.ProcessAllControls();

	reverseButton.Debounce();
	if (reverseButton.RisingEdge()) {
		isReverseInverted = !isReverseInverted;
		isConfigChanged = true;
	}
	freezeButton.Debounce();
	if (freezeButton.RisingEdge()) {
		isFreezeInverted = !isFreezeInverted;
		isConfigChanged = true;
	}

	bool reverseGateState = hw.GetGateState(GATE_REVERSE);
	bool freezeGateState = hw.GetGateState(GATE_FREEZE);
	isReverseActive = isReverseInverted ? !reverseGateState : reverseGateState;
	isFreezeActive = isFreezeInverted ? !freezeGateState : freezeGateState;

	for (size_t i = 0; i < size; i++) {
		if (leftSignal.size() == DFT_SIZE) {
			// remove oldest value from running total
			leftInTotal -= fabsf(leftSignal[0]);
			rightInTotal -= fabsf(rightSignal[0]);
		}
		leftInTotal += fabsf(in[0][i]);
		rightInTotal += fabsf(in[1][i]);
		// store new values
		leftSignal.put(in[0][i]);
		rightSignal.put(in[1][i]);
	}

	// Warp Knob - coarse frequency
	float baseFrequency = fmap(hw.GetKnobValue(KNOB_WARP), FREQUENCY_MIN, FREQUENCY_MAX, Mapping::LOG);
	// Time Knob - fine frequency (10% range of base)
	baseFrequency += 0.1 * baseFrequency * fmap(hw.GetKnobValue(KNOB_TIME), -1.0, 1.0);
	// Time CV - linear FM (20% range of post-fine-adjustment freq)
	baseFrequency += 0.2 * baseFrequency * hw.GetCvValue(CV_TIME);
	// Warp CV - exponential FM
	baseFrequency *= pow(2, hw.GetWarpVoct() / 10.0); // -2^6 to 2^6 of post-linear-FM frequency

	float rawResonance = hw.GetKnobValue(KNOB_BLUR) + hw.GetCvValue(CV_BLUR);
	float resonance = fclamp(rawResonance, 0.0, 2.0);
	float selfOscillation = isFreezeActive ? resonance : fmap(4.0*(resonance - 0.75), 0.0, 1.0);
	selfOscillation = pow(selfOscillation, 2.0); // quadratic curve

	float rawStride = hw.GetKnobValue(KNOB_REFLECT) + hw.GetCvValue(CV_REFLECT);
	float strideFactor = (isReverseActive ? -0.1 : 1) * fmap(rawStride, 0.0, 5.0, Mapping::LINEAR);

	float rawLevel = hw.GetKnobValue(KNOB_ATMOSPHERE) + hw.GetCvValue(CV_ATMOSPHERE);
	float levelFactor = fmap(rawLevel, 0.0, 1.0, Mapping::LINEAR);

	float mix = fclamp(hw.GetKnobValue(KNOB_MIX) + hw.GetCvValue(CV_MIX), 0.0, 1.0);

	// handle self-resonance
#if LOG_ENABLED
	uint32_t setupEnd = timer.GetTick();
	setupTotal += (setupEnd - setupStart) / usecFreq;
	uint32_t resonanceStart = timer.GetTick();
#endif
	memset(leftResonance, 0, sizeof(double)*AUDIO_BLOCK_SIZE);
	memset(rightResonance, 0, sizeof(double)*AUDIO_BLOCK_SIZE);
	double freq = baseFrequency;
	double level = 1.0;
	double levelTotal = 0.0;
	for (size_t i = 0; i < OSCILLATOR_COUNT; i++) {
		Oscillator& l = leftOscillators[i];
		Oscillator& r = rightOscillators[i];
		l.SetFreq(freq);
		r.SetFreq(freq);
		for (size_t j = 0; j < size; j++) {
			leftResonance[j] += level * l.Process();
			rightResonance[j] += level * r.Process();
		}
		levelTotal += level;
		freq += freq * strideFactor;
		float lowFreqDropoff = fclamp(inverse_lerp(freq, HARMONIC_MIN, HARMONIC_DROP_LOW), 0.f, 1.f);
		float highFreqDropoff = fclamp(inverse_lerp(freq, HARMONIC_MAX, HARMONIC_DROP_HIGH), 0.f, 1.f);
		level *= levelFactor * min(lowFreqDropoff, highFreqDropoff);
	}
#if LOG_ENABLED
	uint32_t resonanceEnd = timer.GetTick();
	resonanceTotal += (resonanceEnd - resonanceStart) / usecFreq;
	uint32_t processStart = timer.GetTick();
#endif
	processSignals(baseFrequency, strideFactor, levelFactor, resonance);
#if LOG_ENABLED
	uint32_t processEnd = timer.GetTick();
	processTotal += (processEnd - processStart) / usecFreq;
	uint32_t outputStart = timer.GetTick();
#endif
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		// get outputs
		double leftRaw = leftSignalBuffer[index];
		double rightRaw = rightSignalBuffer[index];
		dft_t previousLeft = 0.0;
		dft_t previousRight = 0.0;
		if (leftOuts.size() > 0) {
			previousLeft = leftOuts[leftOuts.size()-1];
			previousRight = rightOuts[rightOuts.size()-1];
		}
		// adjust level for theoretical max of all haromincs (so level is 1 when all waves coincide)
		double leftRes = selfOscillation * leftResonance[i] / levelTotal;
		double rightRes = selfOscillation * rightResonance[i] / levelTotal;
		dft_t leftValue = limit(leftRaw + leftRes, previousLeft);
		dft_t rightValue = limit(rightRaw + rightRes, previousRight);
		// store result
		if (leftOuts.size() == DFT_SIZE) {
			// remove oldest value from running total
			leftOutTotal -= fabsf(leftOuts[0]);
			rightOutTotal -= fabsf(rightOuts[0]);
		}
		leftOutTotal += fabsf(leftValue);
		rightOutTotal += fabsf(rightValue);
		leftOuts.put(leftValue);
		rightOuts.put(rightValue);
		// mix with input signal (with same delay)
		out[0][i] = (leftSignal[index] * (1.f - mix)) + (leftValue * mix);
		out[1][i] = (rightSignal[index] * (1.f - mix)) + (rightValue * mix);
	}

#if LOG_ENABLED
	uint32_t outputEnd = timer.GetTick();
	uint32_t callbackEnd = timer.GetTick();
	timer.Stop();
	outputTotal += (outputEnd - outputStart) / usecFreq;
	callbackTotal += (callbackEnd - callbackStart) / usecFreq;
	logIteration++;
	if (logIteration % LOG_ITERATIONS == 0) {
		// log totals
		int setupAvg = setupTotal / LOG_ITERATIONS;
		int resAvg = resonanceTotal / LOG_ITERATIONS;
		int processAvg = processTotal / LOG_ITERATIONS;
		int outputAvg = outputTotal / LOG_ITERATIONS;
		int callbackAvg = callbackTotal / LOG_ITERATIONS;
		snprintf(logFileBuffer, 256, "Setup: %i us, Rez: %i us, Proc: %i us, Out: %i us, Total: %i us\n", setupAvg, resAvg, processAvg, outputAvg, callbackAvg);
		isLogWritePending = true;
		// reset
		logIteration = 0;
		setupTotal = 0;
		resonanceTotal = 0;
		processTotal = 0;
		outputTotal = 0;
		callbackTotal = 0;
	}
#endif
}

void _initOsc(Oscillator& osc, bool isCosine) {
	osc.Init(hw.AudioSampleRate());
	osc.SetWaveform(Oscillator::WAVE_SIN);
	osc.SetFreq(FREQUENCY_MIN);
	osc.SetAmp(1.0);
	if (isCosine) {
		osc.PhaseAdd(0.25);
	}
}

void USBConnectCallback(void* userdata) {
	isUsbConnected = true;
}

void USBDisconnectCallback(void* userdata) {
	isUsbConnected = false;
	isUsbConfigured = false;
}

void USBClassActiveCallback(void* userdata) {
	isUsbConfigured = true;
}

// the following file IO variables are global
// as they can't be declared on the stack
//
// access methods also appear to need to be called
// from main (i.e. not from audio callback)
//
// see: https://forum.electro-smith.com/t/error-reading-file-from-sd-card/3911
FIL file;
FRESULT fileResult;
char configFilePath[100];
char fileLineBuffer[100];

bool loadConfig() {
	fileResult = f_open(&file, configFilePath, (FA_OPEN_EXISTING | FA_READ));
	if (fileResult) {
		// report errors through LEDs
		hw.SetLed(LED_1, fileResult == FR_DISK_ERR, fileResult == FR_INT_ERR, fileResult == FR_NOT_READY);
		hw.SetLed(LED_2, 1.0, 1.0, 1.0); // white
		hw.SetLed(LED_3, fileResult == FR_NO_FILE, fileResult == FR_NO_PATH, fileResult == FR_INVALID_NAME);
		hw.SetLed(LED_4, 1.0, 1.0, 1.0); // white
		hw.SetLed(LED_5, fileResult == FR_DENIED, fileResult == FR_EXIST, fileResult == FR_INVALID_OBJECT);
		hw.WriteLeds();
		return false;
	}

	while (f_gets(fileLineBuffer, sizeof fileLineBuffer, &file)) {
		string line(fileLineBuffer);
		string::size_type pos = line.find("=");
		if (pos == string::npos) continue;

		string key = line.substr(0, pos);
		string value = line.substr(pos+1);
		if (key == CONFIG_REVERSE) {
			isReverseInverted = atoi(value.c_str()) > 0;
		}
		else if (key == CONFIG_FREEZE) {
			isFreezeInverted = atoi(value.c_str()) > 0;
		}
	}
	f_close(&file);
	return true;
}

bool writeConfig() {
	fileResult = f_open(&file, configFilePath, (FA_WRITE | FA_CREATE_ALWAYS));
	if (fileResult) {
		// report errors through LEDs
		hw.SetLed(LED_1, fileResult == FR_DISK_ERR, fileResult == FR_INT_ERR, fileResult == FR_NOT_READY);
		hw.SetLed(LED_2, 1.0, 1.0, 1.0); // white
		hw.SetLed(LED_3, fileResult == FR_NO_FILE, fileResult == FR_NO_PATH, fileResult == FR_INVALID_NAME);
		hw.SetLed(LED_4, 1.0, 1.0, 1.0); // white
		hw.SetLed(LED_5, fileResult == FR_DENIED, fileResult == FR_EXIST, fileResult == FR_INVALID_OBJECT);
		hw.WriteLeds();
		return false;
	}

	f_puts(CONFIG_REVERSE, &file);
	f_puts(isReverseInverted ? "=1\n" : "=0\n", &file);
	f_puts(CONFIG_FREEZE, &file);
	f_puts(isFreezeInverted ? "=1\n" : "=0\n", &file);
	f_close(&file);
	return true;
}

int main(void) {
	hw.Init();

	// Prepare for loading config via USB
	const char* usbPath = fatfs_interface.GetUSBPath();
	snprintf(configFilePath, 100, "%s%s", usbPath, CONFIG_FILE_NAME);
#if LOG_ENABLED
	snprintf(logFilePath, 100, "%s%s", usbPath, LOG_FILE_NAME);
#endif

	// ClassActiveCallback appears to only be called when booting from QSPI (or flash?).
	//
	// In Makefile: APP_TYPE = BOOT_QSPI
	//
	// See: https://forum.electro-smith.com/t/usb-capabilities-patch-sm-and-seed/4337/16
	hw.PrepareMedia(USBConnectCallback, USBDisconnectCallback, USBClassActiveCallback);

	// objects required for signal processing
	dft.Init();
	for (size_t i = 0; i < DFT_SIZE; i++) {
		window[i] = hann(indexPhase(i));
	}
	for (size_t i = 0; i < OSCILLATOR_COUNT; i++) {
		_initOsc(leftOscillators[i], false); // sines
		_initOsc(rightOscillators[i], true); // cosines
	}

	reverseButton = hw.GetButton(SW_REVERSE);
	freezeButton = hw.GetButton(SW_FREEZE);

	// ready to start audio
	hw.SetAudioBlockSize(AUDIO_BLOCK_SIZE);
	hw.StartAudio(AudioCallback);

	Color colorWhite;
	colorWhite.Init(Color::WHITE);
	Color colorOff;
	colorOff.Init(Color::OFF);

	/** Infinite Loop */
	srand(0);
	while (1) {
		usb.Process();
		if (isUsbConfigured && !wasConfigLoadAttempted) {
			bool success = loadConfig();
			if (!success) System::Delay(1000);
			wasConfigLoadAttempted = true; // only try to load once
		}
		else if (isUsbConfigured && isConfigChanged) {
			bool success = writeConfig();
			if (!success) System::Delay(1000);
			isConfigChanged = false; // only try to save changed once
		}
#if LOG_ENABLED
		else if (isLogWritePending) {
			bool success = writeLog();
			if (!success) System::Delay(1000);
			isLogWritePending = false;
		}
#endif
		// Update LEDs
		float leftInAvg = 2.0 * leftInTotal * DFT_SIZE_RECIP;
		float rightInAvg = 2.0 * rightInTotal * DFT_SIZE_RECIP;
		float leftOutAvg = 2.0 * leftOutTotal * DFT_SIZE_RECIP;
		float rightOutAvg = 2.0 * rightOutTotal * DFT_SIZE_RECIP;
		float leftMid = (leftInAvg + leftOutAvg) / 2;
		float rightMid = (rightInAvg + rightOutAvg) / 2;
		hw.SetLed(LED_REVERSE, isReverseActive ? colorWhite : colorOff);
		hw.SetLed(LED_FREEZE, isFreezeActive ? colorWhite : colorOff);
		hw.SetLed(LED_1, leftInAvg, 0.0, 0.0); // red
		hw.SetLed(LED_2, 0.0, leftMid, 0.0); // green
		hw.SetLed(LED_3, 0.5*leftOutAvg, 0.0, leftOutAvg); // purple
		hw.SetLed(LED_4, 0.5*rightOutAvg, 0.0, rightOutAvg); // purple
		hw.SetLed(LED_5, 0.0, rightMid, 0.0); // green
		hw.SetLed(LED_6, rightInAvg, 0.0, 0.0); // red
		hw.WriteLeds();
	}
}
