# Arctos GRBL for the Arduino Mega 2560

This repository contains the firmware installed on Peter's Arctos arm. The
`fix/homing-initialization` changes repair system-motion position initialization,
the homing axis counter, and stale inactive-axis step masks. They do not change
pins, COREBC, microstepping, calibration defaults, EEPROM layout, homing input
selection, or the configured Z-then-X/Y full homing sequence.

## What changed

- Initialize all six system-motion starting positions before converting B/C motor
  coordinates. Previously COREBC left X/Y/Z/A positions uninitialized during
  homing, even though ordinary jogs initialized them correctly.
- Reset the active-axis counter at the beginning of every homing pass. Previously
  it was uninitialized, then accumulated through search and locating passes,
  corrupting the speed multiplier.
- Clear every axis's homing step mask at the start of each pass, including axes
  outside the current homing request.
- Identify the patched build as `20261016-homing1` in `$I` output.

These defects can explain unpredictable homing. Software checks cannot establish
that they caused a particular electrical current surge or validate physical motion.

## Build and checks

Install `gcc-avr`, `avr-libc`, and `binutils-avr`, then run:

```sh
python3 -m unittest discover -s tests -v
bash scripts/build-mega.sh
```

The build produces `build/mega/grbl.elf` and `grbl.hex` for an ATmega2560 at 16 MHz.
The script does not upload or erase EEPROM. Existing warnings elsewhere in the
upstream parser are not addressed by this homing patch.

The host tests compile the actual `limits_go_home()` function with simulated
limit/planner/stepper boundaries and the actual planner position initialization
block. They cover X/Y/Z separately, simultaneous X/Y, both homing directions,
search/locating rates, inactive-axis masks, reset, a missing endstop, and normal
jog position initialization. They do not simulate AVR interrupts, step timing,
electrical behavior, or arm mechanics. B/C differential homing is not validated.

## Install and commission

1. Save the full current `$$` and `$I` responses in
   `settings/grbl-settings-2026-10-16.txt` before uploading. The committed snapshot
   has `$24=15`, `$25=60`, `$23=0`, and `$27=5`. Do not restore defaults with `$RST`.
2. Install this patched source as the Arduino IDE's `grbl` library, replacing the
   previous copy rather than leaving duplicate libraries. Open
   `examples/grblUpload/grblUpload.ino`, select Arduino Mega 2560 and the correct
   port, then Verify and Upload with motor outputs disabled and the arm supported.
3. Reconnect at 115200 baud. Send `$I` and confirm `20261016-homing1`; compare `$$`
   with the saved settings. Opening a GUI or changing libraries alone does not
   update firmware on the Mega. Clear the controller app's old captured zero.
4. With motors disabled, verify the actual homing input, not just `Pn:X`. With
   `$5=0`, status combines MIN and MAX, while homing watches MAX only:

   | Axis | MAX / homing pin | MIN pin |
   | --- | --- | --- |
   | X | D2 | D3 |
   | Y | D15 | D14 |
   | Z | D19 | D18 |

   The selected Hall output must pull the MAX pin LOW at home, then release it
   HIGH away from home. WSH231 S_OUT and N_OUT detect opposite magnet polarities;
   use the output the installed home magnet actually activates. Do not change
   `$5` merely to compensate for a magnet triggering the other output.
5. Keep the conservative commissioning values `$25=60` and `$24=15`; verify
   direction `$23`, pull-off `$27`, travel `$130`-`$132`, and calibration before
   movement. These rates are degrees/minute only with correctly calibrated axes.
   The patch does not set any EEPROM values automatically.
6. Start X near its sensor with clearance for approach and pull-off, stabilize the
   motor supply before enabling outputs, and send `$HX` only. Keep reset and
   motor disable accessible. Support the arm before disabling holding torque.
   Abort immediately on wrong direction, a stall, or supply-voltage collapse;
   do not raise VREF or the bench current limit to force homing through a fault.
7. After a successful X cycle, commission `$HY` and `$HZ` separately. Full `$H`
   homes Z followed by X/Y together and should wait until individual tests pass.

During homing this firmware does not answer `?`; silence does not establish a
crash or completion. Abort with the board Reset button or realtime Ctrl-X byte
`0x18` from a sender that supports it. Ordinary feed hold/jog cancel does not
abort this homing loop. After reset or interrupted homing, positions are not
trustworthy; reconnect and establish a fresh reference before jogging.

## Saved settings observations

The committed snapshot has homing enabled (`$22=1`) but both soft limits (`$20=0`)
and hard limits (`$21=0`) disabled. Homing still reads the endstop inputs; those
settings do not provide travel protection during ordinary jogging. No settings
are changed by this patch.

X maximum travel is `$130=400` coordinate degrees. The first homing search can
therefore request up to 600 degrees (1.5 times configured travel) if it does not
find the sensor. Y/Z each allow a 300-degree search from their saved 200-degree
travel. Check these against actual mechanical/cable travel; do not let a missing
sensor run to the full search distance on an assembled arm. The saved `$27=5`
requires clearance for a 5-degree pull-off.
