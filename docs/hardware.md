# Historical hardware setup

The original laboratory experiment used a Phantom/OpenHaptics master, a
three-joint slave/replica, and force sensing. The imported host configuration
used Windows serial ports COM4 and COM7 at 230400 baud and nominal 1 ms control
timing. These are historical setup values, not measured worst-case timing.

The host expected fixed 18-byte frames:

| Bytes | Meaning |
|---|---|
| 0 | STX `0x02` |
| 1 | Frame length `18` |
| 2 | Status byte |
| 3–14 | Three float32 joint positions/torque commands, or force-sensor payload |
| 15–16 | CRC-16/XMODEM over bytes 0–14, little-endian CRC |
| 17 | ETX `0x03` |

The imported Teensy 4.0 teaching sketch instead prints comma-separated ASCII at
115200 baud and contains a zero-torque controller placeholder. It is **not the
matching slave firmware**. The actual binary slave and force-sensor firmware is
unavailable.

At the author's request, files written by other authors and third-party source
are excluded from this public Git repository to avoid authorship and
redistribution conflicts. This includes the KAIST laboratory C++ scaffolding,
Teensy teaching templates, bundled Eigen/serial/filter libraries, and vendor
SDK artifacts. These exclusions do not imply that the original files were
written by Luca Obwegs. The original local import is retained unchanged and
ignored by Git.

Consequently, **this repository is not a runnable hardware-control package**.
It reproduces the author's simulation work and supplies new offline utility
tests. No public executable commands a robot, and no replacement device
firmware or motor calibration has been invented. Bringing the historical rig
back online requires obtaining the correct authorized host/device software,
verifying wiring, calibration, units and timing, and revalidating it on the
actual rig. The original hardware setup has not been altered.
