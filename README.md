# Harmonic Shift Operator

A stereo, self-oscillating spectral filter firmware for the Eurorack
module [Aurora](https://www.qubitelectronix.com/shop/p/aurora) by Qu-Bit
Electronix, inspired by the [Harmonic Shift
Oscillator](https://nsinstruments.com/modules/HSO.html) by New Systems
Instruments.

## About This Firmware

As with the original HSO, a frequency can be set (Warp knob), along with a
stride (Reflect) and level (Atmosphere).  Stride indicates a ratio of the
frequency to the distance between additional frequencies ("harmonics") to pass
through the filter.  Level indicates a ratio between the relative amplitudes
given to each of these frequencies.  To achieve this, each of the input
signals is processed with an [FFT](https://en.wikipedia.org/wiki/Fast_Fourier_transform),
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

The frequency can be modulated via exponential FM (Warp CV input, which tracks
V/oct with a range of +5/-5 octaves), and linear FM (Time CV input, with a range
of 20% of the frequency).

The resonance control (Blur), boosts the level of the base frequency components
and, if level is above zero, proportionally boosts the following frequencies as
well.  When the knob is past 75%, "self-oscillation" will occur, outputting
waves similar to the original HSO, controlled by stride and level as expected.

With stride at 0 and level at 1, all frequencies above the frequency are passed,
creating a kind of brick-wall high-pass filter.

The reverse button toggles between this standard mode and a reversed mode in
which stride becomes negative and targets sub-harmonics instead.  With stride at
0 and level at 1, all frequencies below the frequency are passed, creating a
brick-wall low-pass filter.  This can also be achieved by sending negative CV
values to the stride (Reflect) CV input.  This is added to the value of the
Reflect knob and if the result is negative (or if it is positive and reverse is
active), sub-harmonics will be generated.

The freeze button forces this filter mode and frees the stride and level
controls to only affect the self-oscillation.  This provides a large amount of
control over the resonant character of the filter.

The states of the reverse and freeze buttons can be temporarily toggled with
gates to their respective CV inputs.  The last user-selected state is the
default, and a high signal to the CV switches to the opposite state until the
signal goes low again.  These user-selected reverse and freeze states are also
saved to the USB drive (assuming it is present) to a file called `HSO.txt`,
and loaded on startup.

## Use Cases

### (In-)Harmonic Wave Extractor

Ensure that the freeze control is off (not lit up).  Send a signal to the left
(or left and right) inputs.  Depending on your signal, and the values of the
stride (Reflect) and level (Atmosphere) controls, you may need to adjust the
frequency until you hear any output.

Use the mix knob to fade from the input signal to the processed signal.  When
mix is fully CCW, only the input signal will be output.  When fully CW, only the
processed signal is output.  Note that because the processing requires a
buffered signal, the processed signal is delay relative to the true input.
However, the fade here uses an equally delayed input signal to ensure that the
two signals are in sync with one another.

### Harmonic Shift Oscillator

Without a signal input, set the resonance (Blur) to its maximum value to trigger
HSO-style self-oscillation, controlled by the frequency (time), stride
(reflect), and level (atmosphere) controls.  The left and right channel outputs
are always 90 degrees out of phase with one another, as with the original HSO.

Between the coarse and fine tune (Warp and Time knobs), and the exponential FM
(Warp CV, V/oct) and linear FM (Time CV) inputs, a wide array of harmonic and
in-harmonic sounds are possible.

### Stereo Brick-wall Filter

Send a signal to the left (or left and right) inputs.  Press the freeze button
to enable filter mode (the button should light up).  For a low-pass filter,
press the reverse button until it is also lit up.  For a high-pass filter, leave
reverse disable.  Now listen to the left and right outputs and adjust the
other controls as desired.

## Installing

Download the bin file (see `Releases` section).  If compiling from source,
follow the steps above and find the binary at `build/AuroraHSO.bin`.

Place the `.bin` file on your Aurora's USB drive (make sure this is the only
`.bin` file in the base of the drive) and start the module with the USB
inserted.  This should trigger the firmware flashing procedure.

## Building

If you would like to make changes to the firmware and build it yourself, you can
follow the below instructions to get started.

When testing your changes, you may want to enable performance logging, which
writes timings of various sections of the code at a regular cadence to the USB
drive in a file called `HSO.log`.  To enable this find the line:

```
#define LOG_ENABLED 0
```

And change the `0` to a `1`. To disable logging again, change it back to a `0`
and rebuild the project.

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
