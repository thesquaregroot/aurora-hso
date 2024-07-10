/** Harmonic Shift Operator
 *
 * Inspired by the New System Instruments Harmonic Shift Oscillator, this extends
 * the ideas of harmonic stride and level into a spectral audio processor.
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

Buffer bufferLeft; // input signal for left channel
Buffer bufferRight; // input signal for right channel

Math<dft_t> math;
ShyFFT<dft_t, DFT_SIZE> dft;
dft_t signalBuffer[DFT_SIZE]; // time-domain buffer
dft_t spectrumBuffer[DFT_SIZE]; // frequency-domain buffer

dft_t indexPhase(size_t index) { return index / (dft_t)(DFT_SIZE-1); }

dft_t window(dft_t phase) {
	return 0.5 * (1 - cos(2 * math.pi() * phase)); // hann
}

void processSignal(const Buffer& buffer) {
	for (int i = 0; i < DFT_SIZE; i++) {
		signalBuffer[i] = buffer[i] * window(indexPhase(i));
	}

	dft.Direct(signalBuffer, spectrumBuffer);

	// zero out frequencies above ~1500Hz (= nyquist / bucket_count * cutoff_bucket = 24000/2048*128)
	for (size_t i = 128; i < DFT_SIZE/2; i++) {
		spectrumBuffer[i] = 0; // real part
		spectrumBuffer[DFT_SIZE/2 + i] = 0; // imaginary part
	}

	dft.Inverse(spectrumBuffer, signalBuffer);
}

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
	hw.ProcessAllControls();

	float mix = hw.GetKnobValue(KNOB_MIX);

	for (size_t i = 0; i < size; i++) {
		bufferLeft.put(in[0][i]);
		bufferRight.put(in[1][i]);
	}

	// process left channel
	processSignal(bufferLeft);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		dft_t value = signalBuffer[index] / DFT_SIZE / window(indexPhase(index)); // correct for DFT and window amplitude
		out[0][i] = (in[0][i] * (1.f - mix)) + (value * mix);
	}
	// process right channel
	processSignal(bufferRight);
	for (size_t i = 0; i < size; i++) {
		size_t index = DFT_SIZE/2 - size/2 + i; // read from center of processed signal
		dft_t value = signalBuffer[index] / DFT_SIZE / window(indexPhase(index)); // correct for DFT and window amplitude
		out[1][i] = (in[1][i] * (1.f - mix)) + (value * mix);
	}
}

int main(void) {
	dft.Init();

	hw.Init();
	hw.SetAudioBlockSize(256);
	hw.StartAudio(AudioCallback);

	/** Infinite Loop */
	while (1) {}
}
