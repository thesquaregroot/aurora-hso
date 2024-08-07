# Harmonic Shift Operator

A stereo, self-oscillating spectral filter firmware for the Eurorack
module [Aurora](https://www.qubitelectronix.com/shop/p/aurora) by Qu-Bit
Electronix, inspired by the [Harmonic Shift
Oscillator](https://nsinstruments.com/modules/HSO.html) by New Systems
Instruments.

## About This Firmware

As with the original HSO, a frequency can be set (Warp/Time), along with a
stride (Reflect) and level (Atmosphere).  Stride indicates a multiplier factor
for the distance between harmonics of the base frequency.  Level indicates a
ratio between the relative amplitudes given to each of these frequencies.  Each
input signal is filtered to include only the frequency components matching the
frequency and stride controls, at levels determined by the level control and the
number of the targeted harmonic.

As an example, say the left channel's input is a sum of five sine waves, at 300
Hz, 400 Hz, 600 Hz, 800 Hz, and 1200 Hz.  Suppose frequency = 300 Hz,
stride = 1, and level = 0.5.  The left output signal should
then contain the full-amplitude 300 Hz wave plus a half-amplitude 600 Hz wave,
plus an eighth-amplitude 1200 Hz wave, summed together.  However, if frequency
is set to 100 Hz, stride is set to 3, and level is set to 1, the output will be
only the full-amplitude 400 Hz wave (100 + 3*100).

Unlike the original HSO, the stride control can go negative, in which case
sub-harmonics are targeted, acting as a multiplier of divisions of the base
frequency.  Given the example above, say frequency = 1200, stride = -1, and
level = 0.25.  The output for would then be the full-amplitude 1200 Hz wave,
a quarter-amplitude 600 Hz wave, and a sixteenth-amplitude 300 Hz wave.

To achieve this processing, each of the input signals is processed with an
[FFT](https://en.wikipedia.org/wiki/Fast_Fourier_transform), to determine the
frequencies that make up in the input.  Each of the target frequencies
(determined by the set frequency and stride) is then scaled (based on the level
and harmonic number) and all other frequency content is removed.

The frequency is controlled using coarse (Warp knob) and fine (Time knob)
controls.  It can be modulated via exponential FM (Warp CV input, which tracks
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
brick-wall low-pass filter.  This can also be achieved without reverse, by
sending a negative CV value to the stride (Reflect) CV input.  In general, the
knob and CV values are added together.  If reverse is active, this value is then
negated.  Sub-harmonics are targeted whenever this final value is negative.

The freeze button activates a more explicit filter mode, disconnecting the
stride and level controls from input processing.  Only the base frequency is
boosted with resonance, other passed frequencies are at unity gain. However,
when resonance is boosted to self-oscillation, the stride and level controls
can still affect the generated wave-shape, allowing for a wide range of possible
sounds, even without modulating the frequency.

The states of the reverse and freeze buttons can be temporarily toggled
with gates to their respective CV inputs.  Pressing the button inverts the way
these gates are interpreted.  The user-selected state is the default, and a high
signal to the CV switches to the opposite state until the signal goes low again.
These user-selected reverse and freeze states are also saved to the USB drive
(assuming it is present) to a file called `HSO.txt`, and loaded on startup.

The mix control works as one might expect, fading from the dry signal (CCW) to
the wet signal (CW).  Because the processing introduces a delay, the input signal
is delayed by the same amount to ensure the signals being mixed are in-sync.

Currently, the shift button does nothing.

Finally the front-panel LEDs show estimates of the signal levels.  Along the
top, the left input signal level is shown in purple and the output level is
shown in green near the bend at the top-right.  In cyan between them is the
average of these two levels.  The same is done for the right input and output
signals going down the right side of the module.  The reverse and freeze button
LEDs are white whenever their respective mode is active, and off when in the
default state (e.g. if the user enables filter mode with the freeze button but
also sends a high gate signal to the input, the light will be off).

## Use Cases

### (In-)Harmonic Wave Extractor

Ensure that the freeze control is off (not lit up).  Send a signal to the left
(or left and right) inputs.  Set stride and level somewhere in the middle of
their ranges.  Adjust the frequency until your hear output and adjust the
parameters until you get something you like.

This can be a great way to turn a noisy source (even white noise) into something
more musical.  Note, however, that the output level may be fairly low, since the
targeted harmonics may already have a low level in the input signal.

It you want to lock onto specific partials of your input, the following process
seems to work best.  Use the coarse frequency (Warp) to find a rough starting
point.  Next adjust stride and level until you're getting close to the output
you want.  For example, the may be some desired partials going in and out.
Finally adjust the fine tuning (Time) until you've zeroed in on an output you
like.

There will likely always be some subtle movement to the sound, especially if
level is fairly high, due to fact that the calculations for higher order
harmonics (or sub-harmonics) are very sensitive to the frequency and stride
values, which subject to noise from their respective controls.  It's best to
view this as a kind of subtle chaotic modulation. :)

### Harmonic Shift Oscillator

Without an input signal, set the resonance (Blur) to its maximum value to
trigger HSO-style self-oscillation, controlled by the frequency (Warp/Time), stride
(Reflect), and level (Atmosphere) controls.  The left and right channel outputs
are always 90 degrees out of phase with one another, as with the original HSO.

Between the coarse and fine tune (Warp and Time knobs), and the exponential FM
(Warp CV, V/oct) and linear FM (Time CV) inputs, a wide array of harmonic and
in-harmonic sounds are possible.

Note however, that due to soft clipping on the outputs, setting resonance all
the way up may introduce some saturation beyond the expected spectrum.  Dialing
back the resonance some (~3 o'clock) avoid this if it is not desired.

### Stereo Brick-wall Filter

Send a signal to the left (or left and right) inputs.  Press the freeze button
to enable filter mode (the button should light up).  For a low-pass filter,
press the reverse button until it is also lit up.  For a high-pass filter, leave
reverse disable.  Now listen to the left and right outputs and adjust the
other controls as desired.

Due to the sharp cutoff, you can often hear individual harmonics from the input
signal drop off or appear as the frequency changes.

## Installing

Download the bin file (see `Releases` section).  If compiling from source,
follow the steps below and find the binary at `build/AuroraHSO.bin`.

Place the `.bin` file on your Aurora's USB drive (make sure this is the only
`.bin` file in the base of the drive) and start the module with the USB
inserted.  This should trigger the firmware flashing procedure and once the
white lights go away you should be good to go.

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
