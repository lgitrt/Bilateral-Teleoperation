# Bilateral Teleoperation

![Position/force exchange and passivity signal flow](docs/control-flow.svg)

**Delayed bilateral force and position control, developed by Luca Obwegs during
a 2023 exchange semester in Prof. Jee-Hwan Ryu's laboratory at KAIST.**

[Watch the historical hardware demonstration and controller animation](https://lucaobwegs.com/phantom-robot-bilateral-teleoperation-control/)

![Four-channel reference-model response](docs/simulation-response.png)

*Reproducible simulation output using the saved four-channel configuration.
This plot is not a hardware measurement.*

## Engineering focus

- Bilateral interaction: operator motion drives a remote slave while contact
  force and motion are returned to the master.
- Time-domain passivity observers/controllers track channel energy and apply
  dissipative corrections when the delayed coupling exhibits an energy deficit.
- MATLAB/Simulink comparisons cover position-position, position-force and
  four-channel architectures.
- A symbolic three-joint rigid-body model derives inertia, Coriolis, gravity,
  forward/inverse kinematics and the end-effector Jacobian.
- Historical experiments also explored adaptive inverse-dynamics, Slotine-Li,
  impedance and sliding-mode control. Those hardware implementations are not
  all reproduced by this public package.

## What can be run

| Component | Requirements | Reproduction scope |
|---|---|---|
| Three bilateral `.slx` models | MATLAB + Simulink | Model execution and finite logged-signal validation |
| Symbolic dynamics + numerical tracking | MATLAB + Symbolic Math Toolbox | Model consistency and ideal inverse-dynamics tracking |
| Native C++ utilities/tests | CMake ≥3.20 + C++17 compiler | Delay, packet/CRC handling, low-pass filtering and scalar passivity calculation |
| Historical robot experiment | Original rig and omitted/missing software | **Not reproducible from this public repository alone** |

**Publication boundary:** files written by other authors and third-party
source have been removed from the public Git payload to avoid authorship and
redistribution conflicts. Lab scaffolding, Teensy templates, vendor libraries,
SDK binaries, papers, CAD, caches, builds, large videos and the raw archive are
excluded. This does not transfer their authorship to Luca Obwegs. The original
local setup remains untouched. See [hardware scope](docs/hardware.md).

## Quick start: MATLAB / Simulink

From the repository root:

```matlab
addpath(fullfile(pwd, 'simulation'));
out = run_model("FourCh_TDPA_TD", 10);
validate_models;
export_results;  % Recreates docs/simulation-response.png
```

Select `"PP_TDPA_TD"` or `"PFmsr_TDPA_TD"` to run the other architectures.
Duration must be finite and positive. The wrapper uses its own absolute model
paths and enables logging without saving changes into the original models.
Close an already-open model first; the wrapper refuses to discard unsaved edits.

For the symbolic robot model:

```matlab
addpath(fullfile(pwd, 'simulation', 'dynamics'));
validate_dynamics;
result = run_robot_simulation(5);
plot(result.time, result.state(:,1:3));
xlabel('Time [s]'); ylabel('Joint angle [rad]');
```

This replaces workspace-dependent teaching scripts with explicit callable
entry points. Symbolic construction is cached within the MATLAB session.
Cartesian dynamics use mm; the IK function accepts metres. See
[control design and units](docs/control-design.md).

## Quick start: native regression tests

No robot, OpenHaptics SDK, Eigen installation or MATLAB is needed:

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

With MinGW, use `cmake -S . -B build -G "MinGW Makefiles"` on a fresh build
directory. GitHub Actions runs the native checks on Windows and Linux; MATLAB
checks are separate and require a licensed installation.

## Validation

Locally verified with MATLAB R2024b and a Windows GCC/CMake build:

- All three Simulink models ran to 2 seconds, checking 82 nonempty finite
  logged signals in total; the four-channel documentation plot was reproduced
  over 10 seconds.
- Robot-model checks cover symmetric positive-definite mass matrices at three
  poses, the Coriolis skew-symmetry identity, finite-difference checks of the
  Jacobian and its time derivative, IK/FK round trips, and ideal
  inverse-dynamics tracking.
- 10,236 native checks cover fixed-delay warm-up/wraparound, zero delay, CRC test
  vectors, corrupt/truncated packet recovery, filter convergence and
  dimensionally consistent scalar dissipation.

PP/PF model algebraic-loop warnings and PF unused observer-port warnings are
retained and visible. Test results do not establish a hardware stability
guarantee, arbitrary-delay robustness or measured real-time deadlines.

## Repository layout

```text
core/include/        New dependency-free offline utilities
simulation/models/   Author-owned bilateral Simulink models
simulation/dynamics/ Symbolic robot derivation and clean numerical entry points
tests/               Native regression tests
docs/                Block diagram, reproduced simulation plot and scope/units
.github/workflows/   Windows/Linux native CI
```

No file larger than 5 MiB is intended for the public source payload. Hardware
videos stay on the portfolio and are linked rather than duplicated here.

## References

- Artigas Esclusa, J. (2014). *Time domain passivity control for delayed
  teleoperation*. [DLR thesis record](https://elib.dlr.de/113216/)
- Farkhatdinov, I., & Ryu, J.-H. (2007). Hybrid position-position and
  position-speed command strategy for bilateral teleoperation of a mobile
  robot. [DOI: 10.1109/ICCAS.2007.4406773](https://doi.org/10.1109/ICCAS.2007.4406773)
