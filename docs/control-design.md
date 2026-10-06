# MATLAB simulation and control derivation

## Bilateral interaction

The master and slave form a coupled system: operator motion is sent to the
slave, and the environment interaction is returned to the master. The
position-position, position-force and four-channel Simulink models explore
different ways to exchange that information.

The local robot loop and the communication channel have different roles.
Local feedback and inverse dynamics control the robot's motion; channel
passivity monitors the energy exchanged through delayed force/motion
signals. The animation shows the four-channel model's scalar master/slave
positions together with its controller-force histories.

## Link kinematics and inertia

For each body, homogeneous transforms locate its centre of mass and rotate
its inertia tensor into the world frame. Linear and angular Jacobians give
the velocities associated with the joint vector $q$:

$$v_i=J_{v,i}(q)\dot q,\qquad
\omega_i=J_{\omega,i}(q)\dot q.$$

The kinetic energy is

$$\mathcal T=\frac12\sum_i
\left(m_i v_i^\mathsf{T}v_i+
\omega_i^\mathsf{T}R_iI_iR_i^\mathsf{T}\omega_i\right)
=\frac12\dot q^\mathsf{T}M(q)\dot q.$$

This leads to the summed inertia matrix documented in the README.
The velocity coupling follows from the Christoffel coefficients:

$$c_{ijk}=\frac12\left(
\frac{\partial M_{ij}}{\partial q_k}+
\frac{\partial M_{ik}}{\partial q_j}-
\frac{\partial M_{jk}}{\partial q_i}\right),\qquad
C_{ij}=\sum_k c_{ijk}\dot q_k.$$

The source's gravity term is the derivative of its signed gravity expression
$P(q)=\sum_i m_i g^\mathsf{T}p_{c,i}(q)$:

$$G(q)=\frac{\partial P}{\partial q},\qquad
M(q)\ddot q+C(q,\dot q)\dot q+G(q)=\tau.$$

The gravity vector in this derivation is $g=[0,0,-9810]^\mathsf{T}$
in mm/s^2. The numerical controller uses the same signed model convention.

## End-effector motion

The last link's forward kinematics supply $x=h(q)$. Differentiation yields

$$J(q)=\frac{\partial h}{\partial q},\qquad
\dot J(q,\dot q)=\sum_i\frac{\partial J}{\partial q_i}\dot q_i.$$

For a nonsingular three-joint position Jacobian, a desired Cartesian
acceleration is converted to a joint acceleration by

$$a_x=\ddot x_d+K_{p,x}(x_d-x)+K_{d,x}(\dot x_d-\dot x),$$

$$J(q)a_q=a_x-\dot J(q,\dot q)\dot q.$$

The $\dot J\dot q$ term matters because the Jacobian changes as the robot
moves. Inverse dynamics then converts $a_q$ into torque. Near a kinematic
singularity, the Cartesian-to-joint mapping becomes ill-conditioned.

## Joint-space simulation

The tracking example integrates the state $z=[q^\mathsf{T},
\dot q^\mathsf{T}]^\mathsf{T}$ using

$$\dot z=
\begin{bmatrix}
\dot q\\
M(q)^{-1}\left(\tau-C(q,\dot q)\dot q-G(q)\right)
\end{bmatrix}.$$

In the implementation, the acceleration is evaluated by solving the linear
system with MATLAB's backslash operator. The controller supplies reference
acceleration plus proportional/derivative error feedback before applying
inertia, Coriolis and gravity compensation.

This example uses the same model for the plant and compensation, making it
an ideal-model illustration. Adaptive control for uncertain parameters and
the hardware interaction loop are explained in
[Hardware controller design](hardware.md).

## Energy and units

At a force/velocity port, power is $p=f^\mathsf{T}v$. Integrating power
over the sample interval gives energy in joules when force is in N and
velocity in m/s. The channel observer tracks energy balance; its controller
adds damping when the available balance is negative.

The symbolic robot derivation uses mm, kg, kg mm^2 and mm/s^2.
Joint angles are radians, and generalized torque converts to N m by
$10^{-6}$. Cartesian inputs to inverse kinematics are in metres, so
positions from the symbolic forward model are divided by $1000$ first.
