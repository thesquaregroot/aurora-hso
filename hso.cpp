/** Harmonic Shift Operator
 *
 * Inspired by the New System Instruments Harmonic Shift Oscillator, this extends
 * the ideas of harmonic stride and level into a spectral audio processor.
 *
 * Controls:
 *	- Warp: Base frequency.
 *	- Time: FM depth.
 *	- Blur: Resonance.
 *	- Reflect: Haromonic stride (distance between harmonics).
 *	- Atomosphere: Harmonic level (relative level of each harmonic).
 *	- Mix: Blend between dry signal and processed signal.
 */
#include "aurora.h"
#include "daisysp.h"
#include "fft/shy_fft.h"

using namespace std;
using namespace stmlib;
using namespace daisy;
using namespace aurora;
using namespace daisysp;

#define DFT_SIZE 4096
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

Hardware hw;

Buffer leftSignal; // input signal for left channel
Buffer rightSignal; // input signal for right channel

Math<dft_t> math;
ShyFFT<dft_t, DFT_SIZE> dft;
dft_t signalBuffer[DFT_SIZE];
dft_t spectrumBuffer[DFT_SIZE];
dft_t processedSpectrumBuffer[DFT_SIZE];

dft_t indexPhase(size_t index) { return index / (dft_t)(DFT_SIZE-1); }

dft_t window(dft_t phase) {
	return 0.5 * (1 - cos(2 * math.pi() * phase)); // hann
}

dft_t* processSignal(const Buffer& buffer, float baseFrequency, float strideFactor, float levelFactor) {
	for (int i = 0; i < DFT_SIZE; i++) {
		signalBuffer[i] = buffer[i] * window(indexPhase(i));
	}
	dft.Direct(signalBuffer, spectrumBuffer);

	memset(processedSpectrumBuffer, 0, sizeof(dft_t)*DFT_SIZE);
	float nyquistLimit = hw.AudioSampleRate()/2.0;
	size_t binCount = DFT_SIZE/2;
	float frequency = baseFrequency;
	float level = 1.0;
	size_t lastBin = -1;
	while (frequency < nyquistLimit) {
		size_t bin1 = frequency / nyquistLimit * binCount;
		if (bin1 == lastBin || level == 0) break; // can happen when stride or level is small
		size_t bin2 = binCount + bin1;
		processedSpectrumBuffer[bin1] = level * spectrumBuffer[bin1]; // real part
		processedSpectrumBuffer[bin2] = level * spectrumBuffer[bin2]; // imaginary part
		// updates for next iteration
		lastBin = bin1;
		frequency += frequency * strideFactor;
		level *= levelFactor;
	}

	dft.Inverse(processedSpectrumBuffer, signalBuffer);
	return signalBuffer;
}

dft_t limit(dft_t value) {
	//value = fclamp(value, -1.0, 1.0);
	//TestFloat(value);
	return value;
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
	hw.ProcessAllControls();

	for (size_t i = 0; i < size; i++) {
		leftSignal.put(in[0][i]);
		rightSignal.put(in[1][i]);
	}

	float frequency = fmap(hw.GetKnobValue(KNOB_WARP), 20, 10000, Mapping::LOG);
	float fmDepth = fclamp(hw.GetKnobValue(KNOB_TIME) + hw.GetCvValue(CV_TIME), -1.0f, 1.0f);
	float fmFactor = pow(2, fmDepth * hw.GetWarpVoct() / 10.0); // -2^6 to 2^6
	frequency = fclamp(frequency * fmFactor, 20, 16000);

	float strideFactor = fmap(hw.GetKnobValue(KNOB_REFLECT), 0.0, 5.0, Mapping::LINEAR);
	float levelFactor = fmap(hw.GetKnobValue(KNOB_ATMOSPHERE), 0.0, 1.0, Mapping::LINEAR);

	float mix = hw.GetKnobValue(KNOB_MIX);

	// process left channel
	dft_t* output = processSignal(leftSignal, frequency, strideFactor, levelFactor);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		dft_t value = output[index] / DFT_SIZE / window(indexPhase(index)); // correct for DFT and window amplitude
		out[0][i] = (leftSignal[index] * (1.f - mix)) + (limit(value) * mix);
	}
	// process right channel
	output = processSignal(rightSignal, frequency, strideFactor, levelFactor);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		dft_t value = output[index] / DFT_SIZE / window(indexPhase(index)); // correct for DFT and window amplitude
		out[1][i] = (rightSignal[index] * (1.f - mix)) + (limit(value) * mix);
	}
}

int main(void) {
	dft.Init();
	hw.Init();

	hw.SetAudioBlockSize(1024);
	hw.StartAudio(AudioCallback);

	/** Infinite Loop */
	while (1) {}
}
