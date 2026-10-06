function [JvcEnd, JvcEnd_dot, s1,s2,s3,s4,s5,e1,e2,e3,e4,e5,theta,theta_dot,COM_x,C,D,G] = Dynamics(theta1,theta2,theta3,theta1_dot,theta2_dot,theta3_dot)
%DYNAMICS Symbolic three-joint Phantom replica kinematics and rigid-body dynamics.
% Lengths are mm, masses kg, inertia kg*mm^2 and gravity mm/s^2.

% COM Inertia tensor, COM, Mass
%Link 0
m0 = 6.532; %[kg]
COM0_body = [4.991; 0.001; 24.255]; %[mm]
I0_body = [[43989.506 0.505 6779.087];
           [0.505 89439.517 -0.482];
           [6779.087 -0.482 119836.209]]; %[kgmm^2]
%Link 1
m1 = 0.53; %[kg]
COM1_body = [-9.604; 15.159; 59.074]; %[mm]
I1_body = [[2541.211 -82.979 -190.902];
           [-82.979 2090.928 -577.713];
           [-190.902 -577.713 1347.574]];%[kgmm^2]
%Link 2
m2 = 0.562; %[kg]
COM2_body = [57.307; 0; -12.27]; %[mm]
I2_body = [[488.786 0 16.774];
           [0 657.771 0];
           [16.774 0 266.18]];%[kgmm^2]
%Link 3
m3 = 0.654; %[kg]
COM3_body = [-42.296; 8.638; -10.508]; %[mm]
I3_body = [[497.923 105.328 -115.872];
           [105.328 2325.533 18.357];
           [-115.872 18.357 1954.363]];%[kgmm^2]
%Link 4
m4 = 0.058; %[kg]
COM4_body = [108.424; 0; 0]; %[mm]
I4_body = [[4.03 0 0];
           [0 400.314 0];
           [0 0 398.071]];%[kgmm^2]
%Link 5
m5 = 0.025; %[kg]
COM5_body = [65.274; 0; 0]; %[mm]
I5_body = [[0.335 -0.115 0];
           [-0.115 61.954 0];
           [0 0 61.955]];%[kgmm^2]

% COM JACOBIAN
theta = [theta1, theta2, theta3];
theta_dot = [theta1_dot, theta2_dot, theta3_dot];


%link1
%FORWARD KINEMATICS to COM1
T_01q = [1 0 0 0;
        0 1 0 0;
        0 0 1 101.5;
        0 0 0 1]; %Transformation from inertial frame to link 1 frame height (no rotation yet)
T_1q1 = [cos(theta1) -sin(theta1) 0 0;
           sin(theta1) cos(theta1) 0 0;
           0 0 1 0;
           0 0 0 1]; %rotation about first actuated angle -> frame 1
T_01 = T_01q * T_1q1; %transformation from inertial frame to frame of link1
T_1dot1com = [1 0 0 COM1_body(1);
              0 1 0 COM1_body(2);
              0 0 1 COM1_body(3);
              0 0 0 1];
T_01com = T_01 * T_1dot1com; %COM of body 1 in inertial frame
Pc1 = T_01com(1:3,4);
Jvc1 = [diff(Pc1, theta1) diff(Pc1, theta2) diff(Pc1, theta3)];
Jw1 = [T_01com(1:3,3) zeros(3,1) zeros(3,1)];

%link2
T_12q = [1 0 0 0;
            0 1 0 0;
            0 0 1 131;
            0 0 0 1];
T_2q2qq = [1 0 0 0;
            0 0 1 0;
            0 -1 0 0;
            0 0 0 1]; %Transformation from frame 1 to frame 2 (rotation so that z is in new rotation axis and translation)
T_2qq2 = [cos(theta2-pi/2) -sin(theta2-pi/2) 0 0;
           sin(theta2-pi/2) cos(theta2-pi/2) 0 0;
           0 0 1 0;
           0 0 0 1]; %rotation about second actuated angle -> frame 2
T_22com = [1 0 0 COM2_body(1);
              0 1 0 COM2_body(2);
              0 0 1 COM2_body(3);
              0 0 0 1];
T_02com = T_01 * T_12q * T_2q2qq * T_2qq2 * T_22com;
Pc2 = T_02com(1:3,4);
Jvc2 = [diff(Pc2, theta1) diff(Pc2, theta2) diff(Pc2, theta3)];
Jw2 = [T_01com(1:3,3) T_02com(1:3,3) zeros(3,1)];

%link 3
T_2qq3 = [cos(theta3) -sin(theta3) 0 0;
           sin(theta3) cos(theta3) 0 0;
           0 0 1 0;
           0 0 0 1]; %rotation about second actuated angle -> frame 2
T_33com = [1 0 0 COM3_body(1);
              0 1 0 COM3_body(2);
              0 0 1 COM3_body(3);
              0 0 0 1];
T_03com = T_01 * T_12q * T_2q2qq * T_2qq3 * T_33com;
Pc3 = T_03com(1:3,4);
Jvc3 = [diff(Pc3, theta1) diff(Pc3, theta2) diff(Pc3, theta3)];
Jw3 = [T_01com(1:3,3) zeros(3,1) T_03com(1:3,3)];

%link4
T_24q = [1 0 0 -33;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];
T_4q4 = [cos(pi/2+theta3-theta2) -sin(pi/2+theta3-theta2) 0 0;
           sin(pi/2+theta3-theta2) cos(pi/2+theta3-theta2) 0 0;
           0 0 1 0;
           0 0 0 1];
T_44com = [1 0 0 COM4_body(1);
           0 1 0 COM4_body(2);
           0 0 1 COM4_body(3);
           0 0 0 1];
T_04com = T_01 * T_12q * T_2q2qq * T_2qq2 * T_24q * T_4q4 * T_44com;
Pc4 = T_04com(1:3,4);
Jvc4 = [diff(Pc4, theta1) diff(Pc4, theta2) diff(Pc4, theta3)];
Jw4 = [T_01com(1:3,3) zeros(3,1) T_03com(1:3,3)];

%link 5
T_35q = [1 0 0 210;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];
T_5q5 = [cos(-(-pi/2+theta3-theta2)) -sin(-(-pi/2+theta3-theta2)) 0 0;
           sin(-(-pi/2+theta3-theta2)) cos(-(-pi/2+theta3-theta2)) 0 0;
           0 0 1 0;
           0 0 0 1];
T_55com = [1 0 0 COM5_body(1);
           0 1 0 COM5_body(2);
           0 0 1 COM5_body(3);
           0 0 0 1];

T_05com = T_01 * T_12q * T_2q2qq * T_2qq3 * T_35q * T_5q5 * T_55com;
Pc5 = T_05com(1:3,4);
Jvc5 = [diff(Pc5, theta1) diff(Pc5, theta2) diff(Pc5, theta3)];
Jw5 = [T_01com(1:3,3) T_02com(1:3,3) zeros(3,1)];

%total COM in x direction
COM_x = (T_02com(1,4)*m2+T_03com(1,4)*m3+T_04com(1,4)*m4+T_05com(1,4)*m5)/(m2+m3+m4+m5);


% LINK MASS FROM CAD MODEL
% TRANSFORM INERTIA TENSOR IN BODY FRAME INTO WORLD FRAME
I1_world = T_01com(1:3,1:3) * I1_body * transpose(T_01com(1:3,1:3));
I2_world = T_02com(1:3,1:3) * I2_body * transpose(T_02com(1:3,1:3));
I3_world = T_03com(1:3,1:3) * I3_body * transpose(T_03com(1:3,1:3));
I4_world = T_04com(1:3,1:3) * I4_body * transpose(T_04com(1:3,1:3));
I5_world = T_05com(1:3,1:3) * I5_body * transpose(T_05com(1:3,1:3));
% DERIVE INERTIA MATRIX
D1 = m1*transpose(Jvc1)*Jvc1+transpose(Jw1)*I1_world*Jw1;
D2 = m2*transpose(Jvc2)*Jvc2+transpose(Jw2)*I2_world*Jw2;
D3 = m3*transpose(Jvc3)*Jvc3+transpose(Jw3)*I3_world*Jw3;
D4 = m4*transpose(Jvc4)*Jvc4+transpose(Jw4)*I4_world*Jw4;
D5 = m5*transpose(Jvc5)*Jvc5+transpose(Jw5)*I5_world*Jw5;
D = vpa(simplify(D1+D2+D3+D4+D5),5);
D_simple = vpa(mapSymType(D, 'vpareal', @(x) piecewise(abs(x)<=1e-10, 0, x)),5);


% DERIVE CHRISTOFFEL SYMBOLS
for i=1:1:3
    for j=1:1:3
        C_1_temp(i,j) = calc_christoffel(D,i,j,1,theta);
    end
end

for i=1:1:3
    for j=1:1:3
        C_2_temp(i,j) = calc_christoffel(D,i,j,2,theta);
    end
end

for i=1:1:3
    for j=1:1:3
        C_3_temp(i,j) = calc_christoffel(D,i,j,3,theta);
    end
end

for j=1:1:3
    C_1(j) = C_1_temp(1,j)*theta_dot(1) + C_1_temp(2,j)*theta_dot(2) + C_1_temp(3,j)*theta_dot(3);
end

for j=1:1:3
    C_2(j) = C_2_temp(1,j)*theta_dot(1) + C_2_temp(2,j)*theta_dot(2) + C_2_temp(3,j)*theta_dot(3);
end

for j=1:1:3
    C_3(j) = C_3_temp(1,j)*theta_dot(1) + C_3_temp(2,j)*theta_dot(2) + C_3_temp(3,j)*theta_dot(3);
end
C = [C_1;
C_2;
C_3];

% Skew symmetry
% Check only possible if theta is function of time
%
% N = diff(D)-2*C;
% N = vpa(simplify(N),5);
% N_simple = vpa(mapSymType(N, 'vpareal', @(x) piecewise(abs(x)<=1e-8, 0, x)),5);
%
% % Check skew-symmetry
% N_simple.' + N_simple

% DERIVE GRAVITY TERM
g = [0 0 -9.81]*10^(3); %mm/s^2
P1 = m1*dot(g,T_01com(1:3,4));
P2 = m2*dot(g,T_02com(1:3,4));
P3 = m3*dot(g,T_03com(1:3,4));
P4 = m4*dot(g,T_04com(1:3,4));
P5 = m5*dot(g,T_05com(1:3,4));

P = P1 + P2 + P3 + P4 + P5;
G = [diff(P,theta1);
diff(P,theta2);
diff(P,theta3);];

% Plot Coordinate Systems
%theta1_val = 0;
%theta2_val = -deg2rad(20);
%theta3_val = 0;%-deg2rad(20);
%plot_CS(theta1,theta2,theta3,theta1_val,theta2_val,theta3_val,T_01,T_12q,T_2q2qq,T_2qq2,T_2qq3,T_24q,T_4q4,T_35q,T_5q5);

% Calculate start and end-point of each link symbolic

T_02 = T_01 * T_12q * T_2q2qq * T_2qq2;
T_03 = T_01 * T_12q * T_2q2qq * T_2qq3;
T_04 = T_01 * T_12q * T_2q2qq * T_2qq2 * T_24q * T_4q4;
T_05 = T_01 * T_12q * T_2q2qq * T_2qq3 * T_35q * T_5q5;

T_se4 = [1 0 0 210;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];
T_se5 = [1 0 0 168;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1];
T_e4 = T_04*T_se4;
T_e5 = T_05*T_se5;

% Start points
s1 = T_01(1:3,4);
s2 = T_02(1:3,4);
s3 = T_03(1:3,4);
s4 = T_04(1:3,4);
s5 = T_05(1:3,4);
% End points
e1 = s2;
e2 = s4;
e3 = s5;
e4 = T_e4(1:3,4);
e5 = T_e5(1:3,4);

JvcEnd = [diff(e5, theta1) diff(e5, theta2) diff(e5, theta3)];
syms q1(t) q2(t) q3(t) t
JvcEnd = subs(JvcEnd, theta1, q1(t));
JvcEnd = subs(JvcEnd, theta2, q2(t));
JvcEnd = subs(JvcEnd, theta3, q3(t));
JvcEnd_dot = diff(JvcEnd, t);
JvcEnd_dot = subs(JvcEnd_dot, diff(q1(t), t), theta1_dot);
JvcEnd_dot = subs(JvcEnd_dot, q1(t), theta1);
JvcEnd_dot = subs(JvcEnd_dot, diff(q2(t), t), theta2_dot);
JvcEnd_dot = subs(JvcEnd_dot, q2(t), theta2);
JvcEnd_dot = subs(JvcEnd_dot, diff(q3(t), t), theta3_dot);
JvcEnd_dot = subs(JvcEnd_dot, q3(t), theta3);
JvcEnd = subs(JvcEnd, q1(t), theta1);
JvcEnd = subs(JvcEnd, q2(t), theta2);
JvcEnd = subs(JvcEnd, q3(t), theta3);

end
