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
 *	- Freeze: Filter cutoff mode. This treats the stride as 0 for input processing.
 */
#include <cstdlib> // for srand & rand
#include "aurora.h"
#include "daisysp.h"
#include "fft/shy_fft.h"

using namespace std;
using namespace stmlib;
using namespace daisy;
using namespace aurora;
using namespace daisysp;

#define SAMPLE_RATE 48000
#define DFT_SIZE 4096
#define OSCILLATOR_COUNT 4
#define OUTPUT_BLEND_SAMPLES 4
#define RANDOM_SAMPLE_COUNT 64
#define FREQUENCY_MIN 20
#define FREQUENCY_MAX 16000
#define HARMONIC_MAX 20000

const float RANDOM_SAMPLE_RECIP = 1.0 / RANDOM_SAMPLE_COUNT;
const int BIN_COUNT = DFT_SIZE / 2;
const double BIN_AMPLITUDE_RECIP = 2.0 / DFT_SIZE;
const double FREQUENCY_TO_BIN = 2.0 / SAMPLE_RATE * BIN_COUNT;
const double BIN_WIDTH = (SAMPLE_RATE / 2.0) / BIN_COUNT;
const float STRIDE_EPSILON = 0.001; // -40dB
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

template<typename T>
inline T lerp(T a, T b, T t) {
	return a + t * (b - a);
}

inline float lerp(float a, float b, float t) {
	return a + t * (b - a);
}

Hardware hw;
Oscillator leftOscillators[OSCILLATOR_COUNT];
Oscillator rightOscillators[OSCILLATOR_COUNT];
Switch reverseButton;
bool isReverseActive = false;
bool isReverseInverted = false; // flips gate interpretation, changed by user input
Switch freezeButton;
bool isFreezeActive = false;
bool isFreezeInverted = false; // flips gate interpretation, changed by user input

Buffer leftSignal; // input signal for left channel
Buffer rightSignal; // input signal for right channel
Buffer leftOuts; // output memory for left channel
Buffer rightOuts; // output memory for right channel

Math<dft_t> math;
ShyFFT<dft_t, DFT_SIZE> dft;
dft_t* window = new dft_t[DFT_SIZE];
dft_t* signalBuffer = new dft_t[2*DFT_SIZE];
dft_t* spectrumBuffer = new dft_t[2*DFT_SIZE];
dft_t* processedSpectrumBuffer = new dft_t[2*DFT_SIZE];

dft_t indexPhase(size_t index) { return index / (dft_t)(DFT_SIZE-1); }
dft_t hann(dft_t phase) { return 0.5 * (1 - cos(2 * math.pi() * phase)); }

void processSignals(float baseFrequency, float strideFactor, float levelFactor, float resonance) {
	dft_t* leftSpectrum = spectrumBuffer;
	dft_t* rightSpectrum = spectrumBuffer+DFT_SIZE;
	for (int i = 0; i < DFT_SIZE; i++) {
		signalBuffer[i] = leftSignal[i] * window[i];
		signalBuffer[DFT_SIZE + i] = rightSignal[i] * window[i];
	}
	dft.Direct(signalBuffer, spectrumBuffer);
	dft.Direct(signalBuffer+DFT_SIZE, spectrumBuffer+DFT_SIZE);

	memset(processedSpectrumBuffer, 0, sizeof(dft_t)*2*DFT_SIZE); // clear any existing spectrum data
	dft_t* leftProcessed = processedSpectrumBuffer;
	dft_t* rightProcessed = processedSpectrumBuffer+DFT_SIZE;

	double frequency = baseFrequency;
	double level = 1.0 + resonance;
	size_t bin1 = frequency * FREQUENCY_TO_BIN;
	size_t bin2 = BIN_COUNT + bin1;
	float frequencyIncrement = frequency * strideFactor;
	if (isFreezeActive || abs(frequencyIncrement) < BIN_WIDTH/2.0 || abs(strideFactor) < STRIDE_EPSILON) {
		// copy contiguous chunks of spectrum
		if (isFreezeActive || levelFactor > 1.0 - LEVEL_EPSILON) {
			// no level adjustments, so we can copy more efficiently (and need to)
			if (isReverseActive) {
				// reverse -- low pass
				size_t binsRemaining = bin1 + 1;
				memcpy(leftProcessed, leftSpectrum, sizeof(dft_t)*binsRemaining);
				memcpy(leftProcessed+BIN_COUNT, leftSpectrum+BIN_COUNT, sizeof(dft_t)*binsRemaining);
				memcpy(rightProcessed, rightSpectrum, sizeof(dft_t)*binsRemaining);
				memcpy(rightProcessed+BIN_COUNT, rightSpectrum+BIN_COUNT, sizeof(dft_t)*binsRemaining);
			}
			else {
				// normal -- high pass
				size_t binsRemaining = BIN_COUNT - bin1;
				memcpy(leftProcessed+bin1, leftSpectrum+bin1, sizeof(dft_t)*binsRemaining);
				memcpy(leftProcessed+bin2, leftSpectrum+bin2, sizeof(dft_t)*binsRemaining);
				memcpy(rightProcessed+bin1, rightSpectrum+bin1, sizeof(dft_t)*binsRemaining);
				memcpy(rightProcessed+bin2, rightSpectrum+bin2, sizeof(dft_t)*binsRemaining);
			}
		}
		else {
			// rely on level decay to bail out in a timely fashion
			int increment = isReverseActive ? -1 : 1;
			while (bin1 > 0 && bin1 < BIN_COUNT) {
				leftProcessed[bin1] = level * leftSpectrum[bin1]; // real part
				leftProcessed[bin2] = level * leftSpectrum[bin2]; // imaginary part
				rightProcessed[bin1] = level * rightSpectrum[bin1]; // real part
				rightProcessed[bin2] = level * rightSpectrum[bin2]; // imaginary part
				level *= levelFactor;
				if (level < LEVEL_EPSILON) break;
				bin1 += increment;
				bin2 += increment;
			}
		}
	}
	else {
		// transfer harmonic bin levels to processed spectrum
		if (levelFactor > 1.0 - LEVEL_EPSILON) {
			// most expensive operation, sparse placement but can't rely on level decay
			int increment = isReverseActive ? -1 : 1;
			while (bin1 > 0 && bin1 < BIN_COUNT) {
				leftProcessed[bin1] = leftSpectrum[bin1]; // real part
				leftProcessed[bin2] = leftSpectrum[bin2]; // imaginary part
				rightProcessed[bin1] = rightSpectrum[bin1]; // real part
				rightProcessed[bin2] = rightSpectrum[bin2]; // imaginary part
				// updates for next iteration
				frequency += frequency * strideFactor;
				size_t nextBin = frequency * FREQUENCY_TO_BIN;
				bin1 = (bin1 == nextBin) ? bin1 + increment : nextBin;
				bin2 = BIN_COUNT + bin1;
			}
		}
		else {
			// sparse placement, but level decay allows us to bail out before too long
			while (bin1 > 0 && bin1 < BIN_COUNT) {
				// transfer harmonic bin levels to processed spectrum
				leftProcessed[bin1] = level * leftSpectrum[bin1]; // real part
				leftProcessed[bin2] = level * leftSpectrum[bin2]; // imaginary part
				rightProcessed[bin1] = level * rightSpectrum[bin1]; // real part
				rightProcessed[bin2] = level * rightSpectrum[bin2]; // imaginary part
				// updates for next iteration
				size_t nextBin = bin1;
				do {
					level *= levelFactor;
					if (level < LEVEL_EPSILON) goto idft; // C++ doesn't have "break 2"
					frequency += frequency * strideFactor;
					nextBin = frequency * FREQUENCY_TO_BIN;
				} while (nextBin == bin1);
				bin1 = nextBin;
				bin2 = BIN_COUNT + bin1;
			}
		}
	}
idft:
	dft.Inverse(leftProcessed, signalBuffer);
	dft.Inverse(rightProcessed, signalBuffer+DFT_SIZE);
}

dft_t limit(dft_t value) {
	return SoftClip(value);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
	hw.ProcessAllControls();

	reverseButton.Debounce();
	if (reverseButton.RisingEdge()) {
		isReverseInverted = !isReverseInverted;
	}
	freezeButton.Debounce();
	if (freezeButton.RisingEdge()) {
		isFreezeInverted = !isFreezeInverted;
	}

	bool reverseGateState = hw.GetGateState(GATE_REVERSE);
	bool freezeGateState = hw.GetGateState(GATE_FREEZE);
	isReverseActive = isReverseInverted ? !reverseGateState : reverseGateState;
	isFreezeActive = isFreezeInverted ? !freezeGateState : freezeGateState;

	for (size_t i = 0; i < size; i++) {
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
	float selfOscillation = fmap(4.0*(resonance - 0.75), 0.0, 1.0);
	selfOscillation = pow(selfOscillation, 2.0); // quadratic curve

	float rawStride = hw.GetKnobValue(KNOB_REFLECT) + hw.GetCvValue(CV_REFLECT);
	float strideFactor = (isReverseActive ? -0.1 : 1) * fmap(rawStride, 0.0, 5.0, Mapping::LINEAR);

	float rawLevel = hw.GetKnobValue(KNOB_ATMOSPHERE) + hw.GetCvValue(CV_ATMOSPHERE);
	float levelFactor = fmap(rawLevel, 0.0, 1.0, Mapping::LINEAR);

	float mix = fclamp(hw.GetKnobValue(KNOB_MIX) + hw.GetCvValue(CV_MIX), 0.0, 1.0);

	// handle self-resonance
	double leftResonance[size]{ 0 };
	double rightResonance[size]{ 0 };
	double freq = baseFrequency;
	float level = 0.5 * selfOscillation; // reduced amplitude to give headroom for harmonics
	for (size_t i = 0; i < OSCILLATOR_COUNT; i++) {
		Oscillator& l = leftOscillators[i];
		Oscillator& r = rightOscillators[i];
		l.SetFreq(freq);
		r.SetFreq(freq);
		for (size_t j = 0; j < size; j++) {
			leftResonance[j] += level * l.Process();
			rightResonance[j] += level * r.Process();
		}
		freq += freq * strideFactor;
		level *= levelFactor;
		if (level < LEVEL_EPSILON || freq > HARMONIC_MAX || freq < FREQUENCY_MIN) break;
	}

	// calculate output
	processSignals(baseFrequency, strideFactor, levelFactor, resonance);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		// get outputs
		double leftRaw = signalBuffer[index] * BIN_AMPLITUDE_RECIP;
		double rightRaw = signalBuffer[DFT_SIZE + index] * BIN_AMPLITUDE_RECIP;
		dft_t leftValue = limit(leftRaw + leftResonance[i]);
		dft_t rightValue = limit(rightRaw + rightResonance[i]);
		if (i < OUTPUT_BLEND_SAMPLES) {
			// blend previous output value to increasing degree
			float blend = (i+1.f)/OUTPUT_BLEND_SAMPLES;
			leftValue = lerp(leftOuts[DFT_SIZE-1], leftValue, blend);
			rightValue = lerp(rightOuts[DFT_SIZE-1], rightValue, blend);
		}
		out[0][i] = (leftSignal[index] * (1.f - mix)) + (leftValue * mix);
		out[1][i] = (rightSignal[index] * (1.f - mix)) + (rightValue * mix);
		leftOuts.put(out[0][i]);
		rightOuts.put(out[1][i]);
	}
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

int main(void) {
	dft.Init();
	for (size_t i = 0; i < DFT_SIZE; i++) {
		window[i] = hann(indexPhase(i));
	}

	hw.Init();
	hw.SetAudioBlockSize(256);

	for (size_t i = 0; i < OSCILLATOR_COUNT; i++) {
		_initOsc(leftOscillators[i], false); // sines
		_initOsc(rightOscillators[i], true); // cosines
	}

	reverseButton = hw.GetButton(SW_REVERSE);
	freezeButton = hw.GetButton(SW_FREEZE);
	Color colorWhite;
	colorWhite.Init(Color::WHITE);
	Color colorOff;
	colorOff.Init(Color::OFF);

	hw.StartAudio(AudioCallback);

	/** Infinite Loop */
	srand(0);
	while (1) {
		float leftInAvg = 0.0;
		float rightInAvg = 0.0;
		float leftOutAvg = 0.0;
		float rightOutAvg = 0.0;
		for (size_t i = 0; i < RANDOM_SAMPLE_COUNT; i++) {
			size_t r = rand() % DFT_SIZE;
			leftInAvg += abs(leftSignal[r]);
			rightInAvg += abs(rightSignal[r]);
			leftOutAvg += abs(leftOuts[r]);
			rightOutAvg += abs(rightOuts[r]);
		}
		leftInAvg *= RANDOM_SAMPLE_RECIP;
		rightInAvg *= RANDOM_SAMPLE_RECIP;
		leftOutAvg *= RANDOM_SAMPLE_RECIP;
		rightOutAvg *= RANDOM_SAMPLE_RECIP;
		float leftMid = (leftInAvg + leftOutAvg) / 2;
		float rightMid = (rightInAvg + rightOutAvg) / 2;
		// set leds
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
