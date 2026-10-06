function result = run_robot_simulation(duration)
%RUN_ROBOT_SIMULATION Joint-space inverse-dynamics tracking, without hardware.
arguments
    duration (1,1) double {mustBeFinite, mustBePositive} = 5
end
model = build_robot_model();
initial = zeros(6,1);
options = odeset("RelTol", 1e-6, "AbsTol", 1e-8, "MaxStep", .02);
[time, state] = ode45(@derivative, [0 duration], initial, options);
assert(all(isfinite(state(:))), "teleop:NonFiniteState", "Non-finite simulated state.");
result.time = time;
result.state = state;
result.reference = .15 * (1 - cos(.5 * time)) * [1, -.5, .25];

    function change = derivative(t, s)
        q = s(1:3);
        dq = s(4:6);
        direction = [1; -.5; .25];
        desired = .15 * (1 - cos(.5 * t)) * direction;
        velocity = .075 * sin(.5 * t) * direction;
        acceleration = .0375 * cos(.5 * t) * direction;
        commanded_acceleration = acceleration + 16 * (desired - q) + 8 * (velocity - dq);
        mass = model.mass(q);
        coriolis = model.coriolis(q, dq);
        gravity = model.gravity(q);
        torque = mass * commanded_acceleration + coriolis * dq + gravity;
        change = [dq; mass \ (torque - coriolis * dq - gravity)];
    end
end
