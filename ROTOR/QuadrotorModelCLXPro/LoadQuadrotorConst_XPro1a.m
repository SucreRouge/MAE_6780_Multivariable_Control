% LoadQuadrotorConsts_XPro1a.m
%
% Load electro-mechanical constants for X-Pro specific parameters
% into general quadrotor model:  EPA_Quadrotor_v2b.mdl
%

load('K.mat');

ga = g;                 % grav. accel. (m/s^2)

if strcmp(choice, 'nom')
    m = m_nom;
    L = l_nom;
    Ixx = Ix.nom;
    Iyy = Iy.nom;
    Izz = Iz.nom;
    Tau = tau.nom;              % Motor/Rotor time const (s)
    Gamma = gamma.nom;
elseif strcmp(choice, 'sample')
    m = m_nom;              % total mass (kg)
    L = usample(lx);              % moment arm CG to rotor (m)
    Ixx = usample(Ix);       % Phi (x-axis) Inertia (kgm^2)
    Iyy = usample(Iy);              % Theta (y-axis) Inertia (kgm^2)
    Izz = usample(Iz);       % Psi (z-axis) Inertia (kgm^2)
    Tau = usample(tau);              % Motor/Rotor time const (s)
    Gamma = usample(gamma);
end
Jr=4.86851*0.1;         % rotor inertia (kgm^2)
% Rotor Speed -to- Lift & Drag Functions:
% Lift (N) = c*(r/s)^2+d*(r/s)+e
c = 0.0002;             % Rotor Speed (r/s)-to-Lift (N), 1st const
d = 0.0071;             % Rotor Speed (r/s)-to-Lift (N), 2nd const
e = -0.2625;            % Rotor Speed (r/s)-to-Lift (N), 3rd const
% Drag (Nm) = f*(r/s)^2+g*(r/s)+h
f = 5e-6;               % Rotor Speed (r/s)-to-Drag (Nm), 1st const
g = 0.0008;             % Rotor Speed (r/s)-to-Drag (Nm), 2nd const
h = -0.0282;            % Rotor Speed (r/s)-to-Drag (Nm), 3rd const

% Specific Motor/Rotor Speed Constants: (r/s) = a*(Volts)+b
a1 = 16.235;            % Motor 1 Volts-to-Rotor Speed (r/s), 1st const
b1 = -0.5036;           % Motor 1 Volts-to-Rotor Speed (r/s), 2nd const
a2 = 17.912;            % Motor 2 Volts-to-Rotor Speed (r/s), 1st const
b2 = -12.728;           % Motor 2 Volts-to-Rotor Speed (r/s), 2nd const
a3 = 17.914;            % Motor 3 Volts-to-Rotor Speed (r/s), 1st const
b3 = -6.5646;           % Motor 3 Volts-to-Rotor Speed (r/s), 2nd const
a4 = 18.161;            % Motor 4 Volts-to-Rotor Speed (r/s), 1st const
b4 = -7.4637;           % Motor 4 Volts-to-Rotor Speed (r/s), 2nd const

co_x=1.7*0;
co_y=1.7*0;
co_z=90.7*0;

c_mi_x=170*0;
c_mi_y=170*0;
c_mi_z=40*0;

% Toy LQR
[K_lqr,S,E] = lqr(A.nom,B.nom,eye(12),eye(4).*100);

% get angular velocity
b = 1;% thrust coefficient
d = 0.5;% drag coefficient
Jac = [b, b, b, b;...
    0, -b, 0, b;...
    -b, 0, b, 0;...
    -d, d, -d, d];
invJac = inv(Jac);

