# Hardware controller design

The experiment coupled a Phantom master device to a three-joint slave/replica
with force sensing. The operator commanded motion at the master and felt
remote interaction through force feedback. My work focused on parallel
force/position control with induced communication delay and an adaptive
inverse-dynamics controller, alongside impedance, Slotine-Li adaptive and
sliding-mode experiments.

[Hardware demonstration](https://lucaobwegs.com/phantom-robot-bilateral-teleoperation-control/)

The equations below explain the controller structure and the mathematical
ideas behind the experiments; they are not a listing of device firmware or
its exact gain settings.

## From motion reference to torque

The position channel supplies a delayed reference for the slave. A Cartesian
reference can be mapped to joint angles through inverse kinematics,

$$q_d=h^{-1}(x_d),\qquad x_d(t)=x_m(t-T_{ms}).$$

With a rigid-body model, the local torque command combines desired
acceleration, position/velocity feedback and nonlinear compensation:

$$a_q=\ddot q_d+K_p(q_d-q)+K_d(\dot q_d-\dot q),$$

$$\tau=\hat M(q)a_q+\hat C(q,\dot q)\dot q+\hat G(q).$$

The hats denote estimated model terms. Inertia compensation accounts for
configuration-dependent acceleration loads; the Coriolis term accounts for
velocity coupling, and gravity compensation supports the links without
requiring position error to generate that torque.

## Adaptive model compensation

Robot inverse dynamics can be written as a regressor multiplying a vector
of inertial parameters:

$$M(q)a+C(q,\dot q)v+G(q)=Y(q,\dot q,v,a)\theta.$$

The regressor $Y$ separates measured/reference motion from unknown physical
parameters. Adaptation updates $\hat\theta$ instead of assuming that every
mass, centre of mass and inertia is exactly known.

A Slotine-Li form, also explored in the project, uses

$$e=q-q_d,\qquad v=\dot q_d-\Lambda e,\qquad
a=\ddot q_d-\Lambda\dot e,\qquad r=\dot q-v,$$

$$\tau=Y(q,\dot q,v,a)\hat\theta-K_r r,\qquad
\dot{\hat\theta}=-\Gamma Y^\mathsf{T}r.$$

Here $\Lambda$, $K_r$ and $\Gamma$ are positive-definite design matrices.
The $K_r$ term damps the filtered tracking error; the update moves the
estimated parameters in response to that error. This expresses the adaptive
control idea without confusing model adaptation with communication-channel
passivity: the two act on different parts of the teleoperation loop.

## Force reflection and channel passivity

Position feedback makes the slave follow the operator. Force feedback makes
remote contact perceptible at the master. A Cartesian interaction force is
mapped to joint torques by virtual work:

$$\tau_f=J(q)^\mathsf{T}f,\qquad
\tau_f^\mathsf{T}\dot q=f^\mathsf{T}\dot x.$$

The opposite port directions require consistent master/slave force signs.
The time-domain passivity layer observes $f^\mathsf{T}v$ at the channel ports
and adds a dissipative correction when the delayed exchange has an energy
deficit. The local robot controller shapes motion; the passivity layer shapes
the energy exchanged through the communication channel.

## Impedance control

An impedance objective specifies the desired mechanical relationship
between displacement and external force:

$$M_d\ddot e_x+D_d\dot e_x+K_de_x=f_{\mathrm{ext}},
\qquad e_x=x-x_d.$$

Here $M_d$, $D_d$ and $K_d$ are desired Cartesian inertia, damping and
stiffness; $K_d$ in this equation denotes stiffness, not the joint
derivative gain. Instead of rigidly enforcing a position, the controller
allows compliant motion under contact. A task-space acceleration command
can be mapped back through the Jacobian and the robot dynamics.

## Sliding-mode control

Sliding-mode control uses a combined position/velocity error surface,

$$s=\dot e+\Lambda e,$$

and a corrective term opposing departure from it. A typical smoothed
switching term is

$$\tau_{\mathrm{sw}}=-K_s\,\mathrm{sat}(s/\phi).$$

The boundary layer $\phi$ trades sharp error correction against chattering.
This was an alternative control approach explored alongside model-based
and impedance control; it is distinct from the energy-based passivity
correction used for delayed bilateral interaction.
