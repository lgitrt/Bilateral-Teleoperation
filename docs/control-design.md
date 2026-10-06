# Control design and reproduction boundaries

## Delayed bilateral control

The experiment couples operator motion at a master device to a remote slave and
returns contact information to the operator. Position tracking alone does not
capture the interaction objective: both motion and force matter. Communication
delay introduces phase lag and can make an otherwise well-behaved coupled loop
generate energy.

The published Simulink models explore three architectures:

| Model | Scope |
|---|---|
| `PP_TDPA_TD.slx` | Position-position coupling with time-domain passivity elements |
| `PFmsr_TDPA_TD.slx` | Position-force coupling with measured-force feedback |
| `FourCh_TDPA_TD.slx` | Four-channel position/force exchange and channel energy accounting |

The saved model configurations are retained. `run_model` only overrides duration
and enables output logging; it does not retune gains, replace delays, or suppress
algebraic-loop diagnostics. Inspect each model's Transport Delay and PC switch
blocks before interpreting an experiment. The PP model contains zero-delay
blocks; not every saved model is a nonzero-delay experiment.

## Energy accounting

For a translational port, instantaneous power is

$$p[k] = f[k]^\mathsf{T} v[k], \qquad E[k] = E[k-1] + \Delta t\,p[k].$$

Forces must be in N, velocities in m/s and energy in J. A passivity observer
compares delayed incoming energy with outgoing energy and energy dissipated by
the controller. For a scalar effort-correction port with negative available
energy $W[k]$, an illustrative damping coefficient is

$$\alpha[k] = \frac{-W[k]}{\Delta t\,v[k]^2}.$$

The correction is applied only for an energy deficit and non-negligible
velocity. The new native utility tests exercise this scalar calculation and
explicit sample-delay semantics. They **do not** execute the original hardware
controller or establish closed-loop stability for the Simulink models.

## Three-joint robot model

The symbolic derivation retains the original CAD-derived masses, link geometry,
and inertia tensors:

$$M(q)\ddot q + C(q,\dot q)\dot q + G(q) = \tau.$$

`Dynamics.m` also derives end-effector position, the Jacobian $J(q)$, and
$\dot J(q,\dot q)$. The source uses mm, kg, kg·mm², and mm/s². Generalized
torque in this convention converts to N·m by multiplying by $10^{-6}$.
Joint angles are radians. `inverse_kinematics` accepts Cartesian positions in
**metres**; tests convert the forward-model result accordingly.

`build_robot_model` supplies explicit vector-argument numerical functions,
avoiding the old scripts' dependency on `matlabFunction`'s inferred scalar
argument ordering. Christoffel derivatives use the actual coordinate vector
rather than hard-coded symbol names; tests check the $\dot M - 2C$
skew-symmetry identity and the Jacobian time derivative.
The standalone tracking example uses

$$\tau = M(q)\left(\ddot q_d + K_p(q_d-q)+K_d(\dot q_d-\dot q)\right)
       + C(q,\dot q)\dot q+G(q).$$

The plant and controller use the same model, so its tiny tracking error is an
**ideal-model consistency check**, not evidence of robustness to uncertainty,
delay, contact, sensor noise, or real hardware. Numerical integration uses a
linear solve, not an explicit matrix inverse.

## Known limitations

- PP and PF reference configurations report algebraic-loop warnings in R2024b.
  PF also has unused observer output ports. These warnings are surfaced rather
  than hidden or "fixed" by adding an unvalidated artificial delay.
- The reproduced four-channel response has substantial early force transients.
  Its saved tuning is retained; finite output is not evidence of a well-tuned
  physical force loop.
- The symbolic model is the historical derivation, not a newly identified
  hardware model. The tests check sampled mass-matrix positivity and symmetry,
  the position Jacobian, IK consistency and ideal tracking.
- Adaptive inverse-dynamics, Slotine-Li, impedance and sliding-mode experiments
  are historical project context; the public quick-start does not reproduce
  every hardware controller or claim that the native tests implement them.
- Original lab host code, device firmware templates, vendor SDKs, papers, CAD,
  raw recordings, and historical device logs are not redistributed.
