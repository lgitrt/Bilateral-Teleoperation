function model = build_robot_model()
%BUILD_ROBOT_MODEL Convert the symbolic derivation to consistent vector APIs.
persistent cached
if ~isempty(cached)
    model = cached;
    return
end
syms q1 q2 q3 dq1 dq2 dq3 real
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
[J, Jdot, ~, ~, ~, ~, ~, ~, ~, ~, ~, x, ~, ~, ~, C, M, G] = ...
    Dynamics(q1, q2, q3, dq1, dq2, dq3);
model.mass = matlabFunction(M, "Vars", {q});
model.coriolis = matlabFunction(C, "Vars", {q, dq});
model.gravity = matlabFunction(G, "Vars", {q});
model.position = matlabFunction(x, "Vars", {q});
model.jacobian = matlabFunction(J, "Vars", {q});
model.jacobian_dot = matlabFunction(Jdot, "Vars", {q, dq});
cached = model;
end
