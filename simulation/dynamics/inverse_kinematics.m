function [q1, q2, q3] = inverse_kinematics(X, Y, Z)
%INVERSE_KINEMATICS Cartesian metres to joint radians, original elbow branch.
arguments
    X (1,1) double {mustBeReal, mustBeFinite}
    Y (1,1) double {mustBeReal, mustBeFinite}
    Z (1,1) double {mustBeReal, mustBeFinite}
end
L1 = 0.1015;
L2 = 0.131;
L4 = 0.21;
L5 = 0.168;

q1 = atan2(Y, X);
R = hypot(X, Y);
a1 = Z - (L1 + L2);
beta = atan2(a1, R);
r = hypot(R, a1);
if r < abs(L4 - L5) || r > L4 + L5
    error("teleop:UnreachablePosition", "Position lies outside the two-link workspace.");
end
gamma_cos = (L5^2 - L4^2 - r^2) / (-2 * L4 * r);
gamma = acos(min(1, max(-1, gamma_cos)));
q3 = -beta - gamma;

alpha_cos = (L4^2 + L5^2 - r^2) / (2 * L4 * L5);
alpha = acos(min(1, max(-1, alpha_cos)));
q2 = q3 - alpha + pi/2;
end
