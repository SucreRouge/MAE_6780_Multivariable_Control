% -------------------------------------------------------------------------
% --------------------------- Initial conditions --------------------------
% -------------------------------------------------------------------------

% ----------------- WP: [0 0 0] & [0 0 30] ------------
% -----------------------------------------------------

load('K.mat');


V1_WP=9.1275;               % WP - hovering
V2_WP=8.955;
V3_WP=8.61;
V4_WP=8.543;

d_omega_R10=9.1275;          % motor 1. rotor
d_omega_R20=8.955;           % motor 2. rotor
d_omega_R30=8.61;            % motor 3. rotor
d_omega_R40=8.543;           % motor 4. rotor


dfi_0=-0;                    % roll rate
fi_0=0;                     % roll 

dtheta_0=0;                 % pitch rate
theta_0=0;                  % pitch 

dpsi_0=0;                   % yaw rate
psi_0=0;                    % yaw 

dx_0=0;                     % linear velocity in x - axis
x_0=0;                      % position in x - axis

dy_0=0;                     % linear velocity in y - axis
y_0=0;                      % position in y - axis

dz_0=0;                     % linear velocity in z - axis
z_0=0;                     % position in z - axis

% state = [x y z xd yd zd phi th psi phid thd psid]
% dx0 = [x_0; y_0; z_0; dx_0; dy_0; dz_0; fi_0; theta_0; psi_0; dfi_0; dtheta_0; dpsi_0] - x_op;
state0 = [x_0; y_0; z_0; dx_0; dy_0; dz_0; fi_0; theta_0; psi_0; dfi_0; dtheta_0; dpsi_0];

total_time = 50;           % total time of simulation


