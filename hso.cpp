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

#define DFT_SIZE 4096
#define OSCILLATOR_COUNT 4
#define RANDOM_SAMPLE_COUNT 128
#define FREQUENCY_INCREMENT_MIN 20
#define FREQUENCY_MIN 20
#define FREQUENCY_MAX 16000
#define HARMONIC_MAX 20000

const float RANDOM_SAMPLE_RECIP = 1.0 / RANDOM_SAMPLE_COUNT;
const int BIN_COUNT = DFT_SIZE / 2;
const double BIN_AMPLITUDE_RECIP = 2.0 / DFT_SIZE;

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

Buffer leftSignal; // input signal for left channel
Buffer rightSignal; // input signal for right channel
Buffer leftOuts; // output signal for left channel (used for LEDs)
Buffer rightOuts; // output signal for right channel (used for LEDs)

Math<dft_t> math;
ShyFFT<dft_t, DFT_SIZE> dft;
dft_t* window = new dft_t[DFT_SIZE];
dft_t* signalBuffer = new dft_t[DFT_SIZE];
dft_t* spectrumBuffer = new dft_t[DFT_SIZE];
dft_t* processedSpectrumBuffer = new dft_t[DFT_SIZE];

dft_t indexPhase(size_t index) { return index / (dft_t)(DFT_SIZE-1); }
dft_t hann(dft_t phase) { return 0.5 * (1 - cos(2 * math.pi() * phase)); }

void processSignal(const Buffer& signal, float baseFrequency, float strideFactor, float levelFactor, float resonance) {
	for (int i = 0; i < DFT_SIZE; i++) {
		signalBuffer[i] = signal[i] * window[i];
	}
	dft.Direct(signalBuffer, spectrumBuffer);

	memset(processedSpectrumBuffer, 0, sizeof(dft_t)*DFT_SIZE);
	float nyquistLimit = hw.AudioSampleRate()/2.0;
	double frequency = baseFrequency;
	float level = 1.0 + resonance;
	size_t lastBin = -1;
	while (frequency < nyquistLimit) {
		size_t bin1 = frequency / nyquistLimit * BIN_COUNT;
		if (bin1 == lastBin || level == 0) break; // can happen when stride or level is small
		size_t bin2 = BIN_COUNT + bin1;
		// include adjacent bins (at lower levels) as freqency increases
		int effectWidth = 1 + fclamp(fastlog10f(frequency)-2, 0.0, 5.0);
		for (int i = -effectWidth/2; i <= effectWidth/2; i++) {
			dft_t subLevel = (effectWidth - abs(i)) / (dft_t)effectWidth;
			processedSpectrumBuffer[bin1] += level * subLevel * spectrumBuffer[bin1 + i] * BIN_AMPLITUDE_RECIP; // real part
			processedSpectrumBuffer[bin2] += level * subLevel * spectrumBuffer[bin2 + i] * BIN_AMPLITUDE_RECIP; // imaginary part
		}
		//processedSpectrumBuffer[bin1] += level * spectrumBuffer[bin1] * BIN_AMPLITUDE_RECIP; // real part
		//processedSpectrumBuffer[bin2] += level * spectrumBuffer[bin2] * BIN_AMPLITUDE_RECIP; // imaginary part
		// updates for next iteration
		lastBin = bin1;
		frequency += frequency * strideFactor;
		level *= levelFactor;
	}

	dft.Inverse(processedSpectrumBuffer, signalBuffer);
}

dft_t limit(dft_t value) {
	return SoftClip(value);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
	hw.ProcessAllControls();

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
	float strideFactor = fmap(rawStride, 0.0, 5.0, Mapping::LINEAR);

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
			// progress from previous frequency to new freqeuncy across each sample
			leftResonance[j] += level * l.Process();
			rightResonance[j] += level * r.Process();
		}
		float freqIncrement = freq * strideFactor;
		freq += freqIncrement;
		if (freqIncrement < FREQUENCY_INCREMENT_MIN) {
			// as frequency approaches previous frequency, force level decrease
			float l = lerp(0.0, levelFactor, (freqIncrement - FREQUENCY_INCREMENT_MIN)/FREQUENCY_INCREMENT_MIN);
			level *= fclamp(l, 0.0, levelFactor);
		}
		else if (freq > HARMONIC_MAX) {
			// next harmonic would be outside of safe range
			level = 0.0;
		}
		else if (freq > FREQUENCY_MAX) {
			// drop harmonic level to zero as frequency approaches upper limit
			float l = lerp(levelFactor, 0.0, (freq - FREQUENCY_MAX)/(HARMONIC_MAX - FREQUENCY_MAX));
			level *= fclamp(l, 0.0, levelFactor);
		}
		else {
			level *= levelFactor;
		}
	}

	// process left channel
	processSignal(leftSignal, baseFrequency, strideFactor, levelFactor, resonance);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		dft_t value = limit(signalBuffer[index] + leftResonance[i]);
		out[0][i] = (leftSignal[index] * (1.f - mix)) + (value * mix);
		leftOuts.put(out[0][i]);
	}
	// process right channel
	processSignal(rightSignal, baseFrequency, strideFactor, levelFactor, resonance);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		dft_t value = limit(signalBuffer[index] + rightResonance[i]);
		out[1][i] = (rightSignal[index] * (1.f - mix)) + (value * mix);
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
		hw.SetLed(LED_1, leftInAvg, 0.0, 0.0); // red
		hw.SetLed(LED_2, 0.0, leftMid, 0.0); // green
		hw.SetLed(LED_3, 0.5*leftOutAvg, 0.0, leftOutAvg); // purple
		hw.SetLed(LED_4, 0.5*rightOutAvg, 0.0, rightOutAvg); // purple
		hw.SetLed(LED_5, 0.0, rightMid, 0.0); // green
		hw.SetLed(LED_6, rightInAvg, 0.0, 0.0); // red
		hw.WriteLeds();
	}
}
