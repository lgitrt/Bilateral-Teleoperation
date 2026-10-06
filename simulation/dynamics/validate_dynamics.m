function validate_dynamics()
%VALIDATE_DYNAMICS Check the symbolic model and numerical tracking entry point.
model = build_robot_model();
points = [0, .1, -.2; 0, -.25, .15; 0, .3, -.1];
for i = 1:size(points, 2)
    q = points(:,i);
    M = model.mass(q);
    assert(all(isfinite(M(:))), "teleop:NonFiniteMass", "Non-finite mass matrix.");
    assert(norm(M - M.', "fro") < 1e-8, "teleop:AsymmetricMass", "Mass matrix is not symmetric.");
    assert(all(eig(M) > 0), "teleop:IndefiniteMass", "Mass matrix is not positive definite.");
    velocity = [.1; -.2; .15];
    C = model.coriolis(q, velocity);
    h = 1e-6;
    Mdot = (model.mass(q + h * velocity) - model.mass(q - h * velocity)) / (2*h);
    skew = Mdot - 2*C;
    residual = norm(skew + skew.', "fro") / max(1, norm(Mdot, "fro"));
    assert(residual < 1e-5, ...
        "teleop:CoriolisMismatch", "Mdot - 2C is not skew-symmetric.");
    J = model.jacobian(q);
    h = 1e-6;
    numeric = zeros(3);
    for axis = 1:3
        delta = zeros(3,1);
        delta(axis) = h;
        numeric(:,axis) = (model.position(q + delta) - model.position(q - delta)) / (2*h);
    end
    assert(norm(J - numeric, "fro") < 1e-5, ...
        "teleop:JacobianMismatch", "Analytic Jacobian differs from finite differences.");
    Jdot = model.jacobian_dot(q, velocity);
    numeric_dot = (model.jacobian(q + h * velocity) - ...
        model.jacobian(q - h * velocity)) / (2*h);
    assert(norm(Jdot - numeric_dot, "fro") < 1e-5, ...
        "teleop:JacobianRateMismatch", "Jacobian time derivative differs from finite differences.");
    x = model.position(q) / 1000;
    [a, b, c] = inverse_kinematics(x(1), x(2), x(3));
    assert(norm(model.position([a; b; c]) / 1000 - x) < 1e-9, ...
        "teleop:InverseMismatch", "Inverse-kinematics round trip failed.");
end
result = run_robot_simulation(2);
error = result.state(:,1:3) - result.reference;
assert(max(abs(error(:))) < 1e-5, ...
    "teleop:TrackingError", "Ideal inverse-dynamics tracking tolerance exceeded.");
fprintf("Dynamics: mass/Jacobian/IK checks at 3 poses; ideal tracking error %.3g rad.\n", ...
    max(abs(error(:))));
end
