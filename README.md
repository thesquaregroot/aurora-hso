# aurora-hso

"Harmonic Shift Operator" firmware for the Eurorack module Aurora by Qu-Bit
Electronix, inspired by the Harmonic Shift Oscillator by New Systems
Instruments, extending the concepts of harmonic stride and level into a
spectral audio processor.

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

