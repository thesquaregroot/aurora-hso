# Harmonic Shift Operator

A stereo, self-oscillating spectral filter firmware for the Eurorack
module [Aurora](https://www.qubitelectronix.com/shop/p/aurora) by Qu-Bit
Electronix, inspired by the [Harmonic Shift
Oscillator](https://nsinstruments.com/modules/HSO.html) by New Systems
Instruments.

## About This Firmware

As with the original HSO, a frequency can be set, along with a stride and level.
Stride indicates a ratio of the frequency to the distance between additional
frequencies ("harmonics") to be collected.  Level indicates a ratio between the
relative amplitudes given to each of these frequencies.  To achieve this, each
of the input signals is processed with an [FFT](https://en.wikipedia.org/wiki/Fast_Fourier_transform),
to determine the frequencies that make up in the input.  Each of the target
frequencies (determined by the set frequency and stride) is then scaled (based
on the set level) and all other frequency content is removed.

As an example, say the left channel's input is a sum of five sine waves, at 300
Hz, 400 Hz, 600 Hz, 800 Hz, and 1200 Hz.  Suppose frequency is set to 300 Hz,
stride is set to 1, and level is set to 0.5.  The left output signal should
then contain the full 300 Hz wave plus a half-amplitude 600 Hz, and an
eighth-amplitude 1200 Hz, summed together.  However, if frequency is set to 100
Hz and level is set to 1, the output will be equivalent the input as all of the
components will be output (all are divisible by 100 Hz) at full amplitude (since
level is 1).

## Building

### Windows / Mac OS

Follow the [Daisy Toolchain installation
instructions](https://github.com/Qu-Bit-Electronix/Aurora-SDK/?tab=readme-ov-file#installing-the-toolchain).
Once you have that installed, run the following commands in a Terminal/Git
window, run `./make.sh`.  After the first time, you can simply run `make` to
compile any changes.

### Linux

Install the packages `dfu-util` and `gcc-arm-none-eabi`
(`gcc-arm-none-eabi-bin` [on Arch](https://madskjeldgaard.dk/posts/daisy-setup/)).
Then within this project run `./make.sh`.  This will compile the Aurora SDK
components as well as this project.  After the first time, you can simply run
`make` to compile any changes.

## Installing

Download the bin file (see `Releases` section).  If compiling from source,
follow the steps above and find the binary at `build/AuroraHSO.bin`.

Place the `.bin` file on your Aurora's USB drive (make sure this is the only
`.bin` file in the base of the drive) and start the module with the USB
inserted.  This should trigger the firmware flashing procedure.

## Use Cases

### (In-)Harmonic Wave Extractor



### Harmonic Shift Oscillator

Without a signal input, setting the resonance (Blur) past 75% will trigger
"self-oscillation," with HSO-style waves being output.  These waves are always
90 degrees out of phase with one another and respond to the
frequency/stride/level controls.

Between the coarse and fine tune knobs (Warp and Time), and the exponential FM
(Warp CV, V/oct) and linear FM (Time CV) inputs, a wide array of harmonic and
in-harmonic sounds are possible without any input.

### Stereo Brick-wall Filter



