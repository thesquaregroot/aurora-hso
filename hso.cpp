/** Harmonic Shift Operator
 *
 * Inspired by the New System Instruments Harmonic Shift Oscillator, this extends
 * the ideas of harmonic stride and level into a spectral audio processor.
 *
 * Controls:
 *	- Warp: Base frequency coarse adjustment.  CV is exponential FM (V/oct, ranging from -5 to +5 octaves).
 *	- Time: Base frequency fine adjusment (10% range of coarse).  CV is linear FM (20% of frequency).
 *	- Blur: Resonance, with self-oscillation after 75% of knob range.
 *	- Reflect: Haromonic stride (distance between harmonics), ranging from 0 to 5.  However, with CV,
 *				stride can go through-zero and become negative (see Reverse).
 *	- Atomosphere: Harmonic level (relative level of each harmonic), ranging from 0 to 1.
 *	- Mix: Blend between dry signal and processed signal.
 *	- Reverse: Changes sign of stride (generate/collect subharmonics instead).
 *	- Freeze: Filter cutoff mode.  Treats the frequency as a cutoff frequency, passing all DFT bins
 *				above (or below if reverse is active), essentially becoming a high-pass (or low-pass)
 *				brick wall filter.  This is the same as when stride is 0 and level is 1.
 */
#include <string>
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
#define OSCILLATOR_COUNT 8

#define FREQUENCY_MIN 20
#define FREQUENCY_MAX 16000

#define HARMONIC_MIN 1
#define HARMONIC_DROP_LOW 5
#define HARMONIC_DROP_HIGH 20000
#define HARMONIC_MAX 24000

#define CONFIG_FILE_NAME "HSO.txt"
#define CONFIG_REVERSE "INVERT_REVERSE"
#define CONFIG_FREEZE "INVERT_FREEZE"

constexpr float SAMPLE_RATE_RECIP = 1.0 / SAMPLE_RATE;
constexpr float DFT_SIZE_RECIP = 1.0 / DFT_SIZE;
constexpr int BIN_COUNT = DFT_SIZE / 2;
constexpr float NYQUIST_FREQUENCY = SAMPLE_RATE / 2.0;
constexpr float FREQUENCY_TO_BIN = BIN_COUNT / NYQUIST_FREQUENCY;
constexpr float BIN_WIDTH = NYQUIST_FREQUENCY / BIN_COUNT;
constexpr float HALF_BIN_WIDTH = BIN_WIDTH / 2.0;
constexpr float BIN_OVERLAP = BIN_WIDTH;

// amount frequency has to change before processing is affected
const float FREQUENCY_EPSILON = HALF_BIN_WIDTH;
// amount stride needs to dip below zero before being treated as negative
const float STRIDE_EPSILON = 0.01;

Hardware hw;
bool isUsbConnected = false;
bool isUsbConfigured = false;
bool wasConfigLoadAttempted = false;
bool isConfigChanged = false;

// USB Drive IO
//
// These variables are global because they won't work
// if they are declared on the stack.  The access methods
// also appear to need to be called from main
// (i.e. not from the audio callback).
//
// see: https://forum.electro-smith.com/t/error-reading-file-from-sd-card/3911
#define IO_BUFFER_SIZE 100
FIL file;
FRESULT fileResult;
char configFilePath[IO_BUFFER_SIZE];
char fileLineBuffer[IO_BUFFER_SIZE];

// Logging
#define LOG_ENABLED 1

#if LOG_ENABLED
#define LOG_FILE_NAME "HSO.log"
#define LOG_ITERATIONS 1000
#define SEC_TO_USEC 1000000
const uint32_t USEC_FREQ = System::GetTickFreq() / SEC_TO_USEC;

uint32_t logIteration = 0;
uint32_t setupTime = 0;
uint32_t resonanceTime = 0;
uint32_t processTime = 0;
uint32_t outputTime = 0;
uint32_t callbackTime = 0;

FIL logFile;
FRESULT logFileResult;
char logFilePath[IO_BUFFER_SIZE];
char logFileBuffer[IO_BUFFER_SIZE];
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

dft_t hann(double phase) { return 0.5 * (1 - cos(2 * M_PI * phase)); }

float oscillatorPhases[OSCILLATOR_COUNT];
float leftResonance[AUDIO_BLOCK_SIZE];
float rightResonance[AUDIO_BLOCK_SIZE];

const Switch* reverseButton;
bool isReverseActive = false;
bool isReverseInverted = false; // flips gate interpretation, changed by user input
const Switch* freezeButton;
bool isFreezeActive = false;
bool isFreezeInverted = false; // flips gate interpretation, changed by user input

// variables for tracking averages (for LEDs)
Buffer leftSignal; // input signal for left channel (also used for mix)
Buffer rightSignal; // input signal for right channel (also used for mix)
Buffer leftOuts; // output signal for left channel
Buffer rightOuts; // output signal for right channel
float leftInTotal = 0;
float rightInTotal = 0;
float leftOutTotal = 0;
float rightOutTotal = 0;

// variables for DFT
DTCMRAM ShyFFT<dft_t, DFT_SIZE> dft;
DTCMRAM dft_t window[DFT_SIZE];

DTCMRAM dft_t signalBuffer[2*DFT_SIZE];
DTCMRAM dft_t spectrumBuffer[2*DFT_SIZE];
DTCMRAM dft_t processedSpectrumBuffer[2*DFT_SIZE];

// helper points to inside buffers
dft_t* leftSignalBuffer = signalBuffer;
dft_t* rightSignalBuffer = signalBuffer + DFT_SIZE;

dft_t* leftSpectrum = spectrumBuffer;
dft_t* leftSpectrumReal = leftSpectrum;
dft_t* leftSpectrumImag = leftSpectrum + BIN_COUNT;
dft_t* rightSpectrum = spectrumBuffer + DFT_SIZE;
dft_t* rightSpectrumReal = rightSpectrum;
dft_t* rightSpectrumImag = rightSpectrum + BIN_COUNT;

dft_t* leftProcessed = processedSpectrumBuffer;
dft_t* leftProcessedReal = leftProcessed;
dft_t* leftProcessedImag = leftProcessed + BIN_COUNT;
dft_t* rightProcessed = processedSpectrumBuffer + DFT_SIZE;
dft_t* rightProcessedReal = rightProcessed;
dft_t* rightProcessedImag = rightProcessed + BIN_COUNT;

/**
 * Absolute value via square and square rooting, but with a small constant added which
 * allows us to avoid zero.  Provides a continuous way to go to through-zero, as long
 * as we account for the original sign elsewhere (which we generally need to do anyway).
 */
inline float absNonZero(const float x, const float epsilon) {
	// fairly linear curve on both sides, but rounded near zero (so that value never becomes zero)
	// note that this is continuous and always positive
	return sqrt(x*x + epsilon*epsilon);
}

/**
 * Calculates a stride value that ensures a partial is in every bin.
 * In particular, calculates s so that the first partial is exactly
 * one bin width away from the base frequency.
 *
 * Given a base frequency, f_0, and FFT bin width, b:
 *	positive stride: f_0 + b = f_0 * (1 + s)  =>  s = b / f_0
 *	negative stride: f_0 - b = f_0 / (1 + s)  =>  s = b / (f_0 - b)
 */
inline float getStrideEpsilon(const float frequency, bool isNegative) {
	return isNegative ?
		BIN_WIDTH / (frequency - BIN_WIDTH) :
		BIN_WIDTH / frequency;
}

/**
 * With low stride values, the partial indexes can be very high.  So when we attempt to
 * compute the level for that index, we are often raising level factor to high powers.
 *
 * powf was too slow and fastpower resulted in glitchiness when level factor was near zero.
 * However, we can take advantage of the facts that the power and base are always positive.
 */
float levelPower(float base, uint32_t power) {
	float result = 1;
	while (power > 0) {
		if (power % 2 == 1) {
			result *= base;
		}
		power /= 2;
		base *= base; // inconsequential if power is now 0, but otherwise it's even, so immediately reduce
	}
	return result;
}

/**
 * Calculates the n-th partial of the base frequency, given the stride factor.
 *
 * When stride is positive, we target multiples of the frequency.
 * With stride = 1, this corresponds to harmonics (integer multiples: 1, 2, 3, etc.) or overtones.
 *
 * When stride is negative, we target divisions of the frequency.
 * With stride = -1, this corresponds to subharmonics (integer submultiples: 1/1, 1/2, 1/3, etc.) or undertones.
 *
 * General formulas:
 *   positive stride: f_n = f_0 * (1 + n*s)
 *   negative stride: f_n = f_0 / (1 - n*s)
 */
inline float getFrequency(const float baseFrequency, const float strideFactor, const size_t index) {
	return (strideFactor < 0) ?
			baseFrequency / (1.0 - index * strideFactor) :
			baseFrequency * (1.0 + index * strideFactor);
}

void processSignals(float baseFrequency, float strideFactor, float levelFactor, float resonance) {
	// create windowed signal in buffer
	for (int i = 0; i < DFT_SIZE; i++) {
		leftSignalBuffer[i] = leftSignal[i] * window[i];
		rightSignalBuffer[i] = rightSignal[i] * window[i];
	}
	// perform forward transform
	dft.Direct(leftSignalBuffer, leftSpectrum);
	dft.Direct(rightSignalBuffer, rightSpectrum);

	float frequency = baseFrequency;
	size_t cutoffBin = round(baseFrequency * FREQUENCY_TO_BIN);

	float baseLevel = 1.0 + resonance;
	if (isFreezeActive) {
		memset(processedSpectrumBuffer, 0, sizeof(dft_t)*2*DFT_SIZE); // clear any existing spectrum data

		// copy specturm without level adjustments
		if (isReverseActive) {
			// reverse -- low pass
			size_t binsRemaining = cutoffBin + 1;
			memcpy(leftProcessedReal, leftSpectrumReal, sizeof(dft_t)*binsRemaining);
			memcpy(leftProcessedImag, leftSpectrumImag, sizeof(dft_t)*binsRemaining);
			memcpy(rightProcessedReal, rightSpectrumReal, sizeof(dft_t)*binsRemaining);
			memcpy(rightProcessedImag, rightSpectrumImag, sizeof(dft_t)*binsRemaining);
		}
		else {
			// normal -- high pass
			size_t binsRemaining = BIN_COUNT - cutoffBin;
			memcpy(leftProcessedReal+cutoffBin, leftSpectrumReal+cutoffBin, sizeof(dft_t)*binsRemaining);
			memcpy(leftProcessedImag+cutoffBin, leftSpectrumImag+cutoffBin, sizeof(dft_t)*binsRemaining);
			memcpy(rightProcessedReal+cutoffBin, rightSpectrumReal+cutoffBin, sizeof(dft_t)*binsRemaining);
			memcpy(rightProcessedImag+cutoffBin, rightSpectrumImag+cutoffBin, sizeof(dft_t)*binsRemaining);
		}
		// boost cutoff freqeuncy (resonance)
		leftProcessedReal[cutoffBin] *= baseLevel;
		leftProcessedImag[cutoffBin] *= baseLevel;
		rightProcessedReal[cutoffBin] *= baseLevel;
		rightProcessedImag[cutoffBin] *= baseLevel;
	}
	else {
		// due to noisy controls, and the fact that crossing zero for processing can have
		// a pretty drastic/sudden effect, this adds some buffer depending on the mode
		// to minimize the likely that processing switching directions unexpectedly
		bool isStrideNegative = isReverseActive ? (strideFactor < STRIDE_EPSILON) : (strideFactor < -STRIDE_EPSILON);

		// since some of the calculations below require dividing by the stride
		// (and one requires a positive version of it), we use a trick to
		// round out the absolute value near zero.  This also provides an opportunity
		// to keep stride from getting too small and blowing things up anyway
		float strideEpsilon = getStrideEpsilon(baseFrequency, isStrideNegative);
		float safeStride = absNonZero(strideFactor, strideEpsilon);
		float signedSafeStride = isStrideNegative ? -safeStride : safeStride;

		// iterating through all bins (except 0 since we're just going to clear it)
		// as this is more efficient/consistent than iterating through partials,
		// particularly when stride is very low and level is high
		for (size_t bin = 1; bin < BIN_COUNT; bin++) {
			float binFrequency = bin * BIN_WIDTH;
			// calculate nearest partial frequency and check if it's in this bin
			// for positive stride,  n = (f_n / f_0 - 1) / s
			// for negative stride,  n = (f_0 / f_n - 1) / -s
			float posFrequencyRatio = binFrequency / frequency;
			float negFrequencyRatio = frequency / binFrequency;
			float numerator = (isStrideNegative ? negFrequencyRatio : posFrequencyRatio) - 1.0f;
			uint32_t nearestPartialIndex = round(numerator / safeStride);
			float nearestPartialFrequency = getFrequency(frequency, signedSafeStride, nearestPartialIndex);
			// determine the bin level using the partial's nearness to the bin frequency
			float distanceToBin = fabsf(nearestPartialFrequency - binFrequency) / BIN_OVERLAP;
			float proximityLevel = fclamp(1.0f - distanceToBin, 0, 1); // 1 when at bin, down to zero at overlap distance
			float binLevel = (nearestPartialIndex >= 0) * baseLevel * proximityLevel * levelPower(levelFactor, nearestPartialIndex);
			// transfer harmonic bin levels to processed spectrum
			leftProcessedReal[bin] = binLevel * leftSpectrumReal[bin];
			leftProcessedImag[bin] = binLevel * leftSpectrumImag[bin];
			rightProcessedReal[bin] = binLevel * rightSpectrumReal[bin];
			rightProcessedImag[bin] = binLevel * rightSpectrumImag[bin];
		}
	}
	// prevent issues with low frequencies / DC offset
	leftProcessedReal[0] = 0;
	leftProcessedImag[0] = 0;
	rightProcessedReal[0] = 0;
	rightProcessedImag[0] = 0;
	// perform inverse transform
	dft.Inverse(leftProcessed, leftSignalBuffer);
	dft.Inverse(rightProcessed, rightSignalBuffer);
}

inline dft_t limit(dft_t value) {
	return SoftClip(value);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
#if LOG_ENABLED
	// get start of overall callback and first section
	uint32_t callbackStart = System::GetTick();
	uint32_t setupStart = System::GetTick();
#endif
	hw.ProcessAllControls();

	if (reverseButton->RisingEdge()) {
		isReverseInverted = !isReverseInverted;
		isConfigChanged = true;
	}
	if (freezeButton->RisingEdge()) {
		isFreezeInverted = !isFreezeInverted;
		isConfigChanged = true;
	}

	bool reverseGateState = hw.GetGateState(GATE_REVERSE);
	bool freezeGateState = hw.GetGateState(GATE_FREEZE);
	isReverseActive = isReverseInverted ? !reverseGateState : reverseGateState;
	isFreezeActive = isFreezeInverted ? !freezeGateState : freezeGateState;

	// Warp Knob - coarse frequency
	float baseFrequency = fmap(hw.GetKnobValue(KNOB_WARP), FREQUENCY_MIN, FREQUENCY_MAX, Mapping::LOG);
	// Time Knob - fine frequency (10% range of base)
	baseFrequency += 0.1 * baseFrequency * fmap(hw.GetKnobValue(KNOB_TIME), -1.0, 1.0);
	// Time CV - linear FM (20% range of post-fine-adjustment freq)
	baseFrequency += 0.2 * baseFrequency * hw.GetCvValue(CV_TIME);
	// Warp CV - exponential FM
	baseFrequency *= powf(2.0, hw.GetWarpVoct() / 12.0); // -2^5 to 2^5 of post-linear-FM frequency

	// Blur Knob/CV - resonance
	float rawResonance = hw.GetKnobValue(KNOB_BLUR) + hw.GetCvValue(CV_BLUR);
	float resonance = fclamp(rawResonance, 0.0, 2.0);
	float selfOscillation = fmap(4.0*(resonance - 0.75), 0.0, 1.0);
	selfOscillation = selfOscillation * selfOscillation;

	// Reflect Knob/CV - stride
	float rawStride = hw.GetKnobValue(KNOB_REFLECT) + hw.GetCvValue(CV_REFLECT);
	float strideFactor = (isReverseActive ? -5.0 : 5.0) * rawStride;

	// Atmosphere Knob/CV - level
	float rawLevel = hw.GetKnobValue(KNOB_ATMOSPHERE) + hw.GetCvValue(CV_ATMOSPHERE);
	float levelFactor = fmap(rawLevel, 0.0, 1.0, Mapping::LINEAR);

	// Mix Knob/CV - mix
	float mix = fclamp(hw.GetKnobValue(KNOB_MIX) + hw.GetCvValue(CV_MIX), 0.0, 1.0);

	// Collect input signals
	for (size_t i = 0; i < size; i++) {
		// update running totals
		if (leftSignal.size() == DFT_SIZE) {
			leftInTotal -= fabsf(leftSignal[0]);
			rightInTotal -= fabsf(rightSignal[0]);
		}
		leftInTotal += fabsf(in[0][i]);
		rightInTotal += fabsf(in[1][i]);
		// store new values
		leftSignal.put(in[0][i]);
		rightSignal.put(in[1][i]);
	}

#if LOG_ENABLED
	uint32_t setupEnd = System::GetTick();
	setupTime += (setupEnd - setupStart) / USEC_FREQ;
	uint32_t processStart = System::GetTick();
#endif

	processSignals(baseFrequency, strideFactor, levelFactor, resonance);

#if LOG_ENABLED
	uint32_t processEnd = System::GetTick();
	processTime += (processEnd - processStart) / USEC_FREQ;
	uint32_t resonanceStart = System::GetTick();
#endif

	// handle self-oscillation
	memset(leftResonance, 0, sizeof(float)*AUDIO_BLOCK_SIZE);
	memset(rightResonance, 0, sizeof(float)*AUDIO_BLOCK_SIZE);
	// determine where we might drop partials
	float dropEnd = strideFactor < 0 ? HARMONIC_MIN : HARMONIC_MAX;
	float dropStart = strideFactor < 0 ? HARMONIC_DROP_LOW : HARMONIC_DROP_HIGH;
	// compute sum of oscillations
	//
	// in order to easily/more efficient compute the sine and cosine of the phase,
	// the Oscillator class isn't used, though the logic here is essentially identical
	float level = 1.0;
	float resonanceLevelTotal = 0.0;
	for (size_t i = 0; i < OSCILLATOR_COUNT; i++) {
		float& phase = oscillatorPhases[i];
		float frequency = getFrequency(baseFrequency, strideFactor, i);
		float increment = (TWOPI_F * frequency) * SAMPLE_RATE_RECIP;
		float dropoff = fclamp(inverse_lerp(frequency, dropEnd, dropStart), 0.f, 1.f);
		float partialLevel = level * dropoff;
		for (size_t j = 0; j < size; j++) {
			phase += increment - (phase > TWOPI_F ? TWOPI_F : 0);
			leftResonance[j] += partialLevel * sinf(phase);
			rightResonance[j] += partialLevel * cosf(phase);
		}
		resonanceLevelTotal += level;
		level *= levelFactor;
	}
	float resonanceScale = selfOscillation / resonanceLevelTotal; // adjustment for maximum resonance amplitude

#if LOG_ENABLED
	uint32_t resonanceEnd = System::GetTick();
	resonanceTime += (resonanceEnd - resonanceStart) / USEC_FREQ;
	uint32_t outputStart = System::GetTick();
#endif

	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		// get outputs
		float signalScale = DFT_SIZE_RECIP / window[index]; // adjust for DFT scale and window distortion
		float leftRaw = leftSignalBuffer[index] * signalScale;
		float rightRaw = rightSignalBuffer[index] * signalScale;
		float leftRes = leftResonance[i] * resonanceScale;
		float rightRes = rightResonance[i] * resonanceScale;
		dft_t leftValue = limit(leftRaw + leftRes);
		dft_t rightValue = limit(rightRaw + rightRes);
		// update running totals
		if (leftOuts.size() == DFT_SIZE) {
			leftOutTotal -= fabsf(leftOuts[0]);
			rightOutTotal -= fabsf(rightOuts[0]);
		}
		leftOutTotal += fabsf(leftValue);
		rightOutTotal += fabsf(rightValue);
		// store new values
		leftOuts.put(leftValue);
		rightOuts.put(rightValue);
		// mix with input signal (with same delay)
		out[0][i] = (leftSignal[index] * (1.f - mix)) + (leftValue * mix);
		out[1][i] = (rightSignal[index] * (1.f - mix)) + (rightValue * mix);
	}

#if LOG_ENABLED
	uint32_t outputEnd = System::GetTick();
	uint32_t callbackEnd = System::GetTick();
	outputTime += (outputEnd - outputStart) / USEC_FREQ;
	callbackTime += (callbackEnd - callbackStart) / USEC_FREQ;
	logIteration++;
	if (logIteration % LOG_ITERATIONS == 0) {
		// log totals
		int setupAvg = setupTime / LOG_ITERATIONS;
		int processAvg = processTime / LOG_ITERATIONS;
		int resonanceAvg = resonanceTime / LOG_ITERATIONS;
		int outputAvg = outputTime / LOG_ITERATIONS;
		int callbackAvg = callbackTime / LOG_ITERATIONS;
		snprintf(logFileBuffer, IO_BUFFER_SIZE, "Setup: %i us, Proc: %i us, Rez: %i us, Out: %i us, Total: %i us\n", setupAvg, processAvg, resonanceAvg, outputAvg, callbackAvg);
		isLogWritePending = true;
		// reset
		logIteration = 0;
		setupTime = 0;
		processTime = 0;
		resonanceTime = 0;
		outputTime = 0;
		callbackTime = 0;
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
	snprintf(configFilePath, IO_BUFFER_SIZE, "%s%s", usbPath, CONFIG_FILE_NAME);
#if LOG_ENABLED
	snprintf(logFilePath, IO_BUFFER_SIZE, "%s%s", usbPath, LOG_FILE_NAME);
#endif

	// ClassActiveCallback appears to only be called when booting from QSPI (or flash?).
	//
	// In Makefile: APP_TYPE = BOOT_QSPI
	//
	// See: https://forum.electro-smith.com/t/usb-capabilities-patch-sm-and-seed/4337/16
	hw.PrepareMedia(USBConnectCallback, USBDisconnectCallback, USBClassActiveCallback);

	// objects required for signal processing
	dft.Init();
	memset(window, 0, sizeof(dft_t)*DFT_SIZE);
	for (size_t i = 0; i < DFT_SIZE; i++) {
		window[i] = hann(i / (double)(DFT_SIZE - 1));
	}
	memset(signalBuffer, 0, sizeof(dft_t)*2*DFT_SIZE); // mostly just for easy skipping of process when testing

	reverseButton = &hw.GetButton(SW_REVERSE);
	freezeButton = &hw.GetButton(SW_FREEZE);

	// ready to start audio
	hw.SetAudioBlockSize(AUDIO_BLOCK_SIZE);
	hw.StartAudio(AudioCallback);

	Color colorWhite;
	colorWhite.Init(Color::WHITE);
	Color colorOff;
	colorOff.Init(Color::OFF);

	/** Infinite Loop */
	while (1) {
		usb.Process();
		if (isUsbConfigured) {
			if (!wasConfigLoadAttempted) {
				bool success = loadConfig();
				if (!success) System::Delay(1000);
				wasConfigLoadAttempted = true; // only try to load once
			}
			else if (isConfigChanged) {
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
		}
		// Update LEDs
		float leftInAvg = 2.0 * leftInTotal * DFT_SIZE_RECIP;
		float rightInAvg = 2.0 * rightInTotal * DFT_SIZE_RECIP;
		float leftOutAvg = 2.0 * leftOutTotal * DFT_SIZE_RECIP;
		float rightOutAvg = 2.0 * rightOutTotal * DFT_SIZE_RECIP;
		float leftMid = (leftInAvg + leftOutAvg) / 2;
		float rightMid = (rightInAvg + rightOutAvg) / 2;
		hw.SetLed(LED_REVERSE, isReverseActive ? colorWhite : colorOff);
		hw.SetLed(LED_FREEZE, isFreezeActive ? colorWhite : colorOff);
		// input levels (purple)
		hw.SetLed(LED_1, 0.5*leftInAvg, 0.0, leftInAvg);
		hw.SetLed(LED_4, 0.5*rightInAvg, 0.0, rightInAvg);
		// blend (cyan)
		hw.SetLed(LED_2, 0.0, leftMid, leftMid);
		hw.SetLed(LED_5, 0.0, rightMid, rightMid);
		// output levels (green)
		hw.SetLed(LED_3, 0.0, leftOutAvg, 0.25*leftOutAvg);
		hw.SetLed(LED_6, 0.0, rightOutAvg, 0.25*rightOutAvg);
		hw.WriteLeds();
	}
}
