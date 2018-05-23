%% Quadcopter Robust Control
clear; close all; clc;
% state = [x y z xd yd zd phi th psi phid thd psid]
% control = [u1 u2 u3 u4];


% uncertain parameters
g = 9.807; % m/s^2
m_nom = 2.1; % kg
l_nom = 0.422; % m
tau_nom = 0.001; % s 0.1
gamma_nom = 0.001; 
lx = ureal('lx', l_nom, 'Percentage', 30); % m
ly = ureal('ly', l_nom, 'Percentage', 30); % m
m = ureal('m', m_nom, 'Percentage', 30); % kg
Ix = ureal('Ix', 2.1385, 'Percentage', 30); % kg m^2
Iy = ureal('Iy', 2.1385, 'Percentage', 30); % kg m^2
Iz = ureal('Iz', 3.7479, 'Percentage', 30); % kg m^2
tau = ureal('tau', tau_nom, 'Percentage', 30); 
gamma = ureal('gamma', gamma_nom, 'Percentage', 30);
% operating point
U1_op = m_nom*g;
psi_op = 0;
phi_op = 0;
th_op = 0;
x_op = [0; 0; 0; 0; 0; 0; phi_op; th_op; psi_op; 0; 0; 0];
u_op = [U1_op; 0; 0; 0];

% linearized dynamics
A = umat(zeros(12, 12)); 
A(1:3, 4:6) = eye(3); 
A(4:5, 7:8) = [sin(psi_op)*U1_op/m, cos(psi_op)*U1_op/m;...
              -cos(psi_op)*U1_op/m, sin(psi_op)*U1_op/m]; 
A(7:9, 10:12) = eye(3);
B = umat(zeros(12, 4));
B(4, 1) = (phi_op*sin(psi_op) + th_op*cos(psi_op))/m;
B(5, 1) = (-phi_op*cos(psi_op) + th_op*sin(psi_op))/m;
B(6, 1) = 1/m;
B(10:12, 2:4) = diag([lx/Ix; ly/Iy; 1/Iz]);
C = eye(12);
states = {'dx', 'dy', 'dz', 'dxd', 'dyd', 'dzd', 'dphi', 'dth', 'dpsi', 'dphid', 'dthd', 'dpsid'};
inputs = {'du1', 'du2', 'du3', 'du4'};
outputs = {'dxo', 'dyo', 'dzo', 'dxdo', 'dydo', 'dzdo', 'dphio', 'dtho', 'dpsio', 'dphido', 'dthdo', 'dpsido'};
P = ss(A, B, C, 0, 'statename', states, 'inputname', inputs, 'outputname', outputs);

% sensor noise
sensor_noise_weights = [0.1, 0.1, 0.1, 0.01, 0.01, 0.01, 0.0873, 0.0873, 0.0873, 0.00873, 0.00873, 0.00873];
Wn = ss(diag(sensor_noise_weights));

% actuator perf
Wc = ss(diag([0.4, 1, 1, 10]));

% actuator lag and time delay
Wact_del = append(tf(1, [tau, 1])*tf([-gamma, 1],[gamma, 1]),...
    tf(1, [tau, 1])*tf([-gamma, 1],[gamma, 1]),...
    tf(1, [tau, 1])*tf([-gamma, 1],[gamma, 1]),...
    tf(1, [tau, 1])*tf([-gamma, 1],[gamma, 1]));

% external input disturbance
input_disturbance = [2, 0.4, 0.4, 0.05];
Wdist = ss(diag(input_disturbance));

% label block I/O's
Wn.u = 'noise'; Wn.y = 'Wn';
Wc.u = {'u1', 'u2', 'u3', 'u4'}; Wc.y = 'z_act';
Wdist.u = 'dist'; Wdist.y = 'Wdist';
Wact_del.u = {'u1', 'u2', 'u3', 'u4'}; Wact_del.y = {'u11', 'u22', 'u33', 'u44'};

% specify summing junctions
Sum1 = sumblk('%u_dist = %u + Wdist', {'du1', 'du2', 'du3', 'du4'}, {'u11', 'u22', 'u33', 'u44'});
Sum2 = sumblk('ymeas = %y + Wn', {'dxo', 'dyo', 'dzo', 'dxdo', 'dydo', 'dzdo', 'dphio', 'dtho', 'dpsio', 'dphido', 'dthdo', 'dpsido'});
Sum3 = sumblk('err = ymeas - dx_des', 12);

% connect everything
% M = connect(P, Wn, Wc, Wdist, Sum1, Sum2, Sum3,...
%     {'dist', 'noise', 'dx_des', 'u1', 'u2', 'u3', 'u4'}, ...
%     {'z_act', 'err1', 'err2', 'err3', 'err4', 'err5', 'err6', 'err7', 'err8', 'err9', 'err10', 'err11', 'err12'});

M = connect(P, Wn, Wc, Wdist, Wact_del, Sum1, Sum2, Sum3,...
    {'dist', 'noise', 'dx_des', 'u1', 'u2', 'u3', 'u4'}, ...
    {'z_act', 'err', 'ymeas'});
% M = connect(P, Wn, Wc, Wdist, Sum1, Sum2,...
%     {'dist', 'noise', 'u1', 'u2', 'u3', 'u4'}, ...
%     {'z_act', 'dxm', 'dym', 'dzm', 'dxdm', 'dydm', 'dzdm', 'dphim', 'dthm', 'dpsim', 'dphidm', 'dthdm', 'dpsidm'});


% D-K iteration
disp('Start D-K');
opts = dksynOptions('MixedMU','on');
[k,clp,bnd,dkinfo] = dksyn(M,12,4,opts);
disp('End D-K');

save('QuadrotorModelCLXPro/K.mat', 'k', 'clp', 'bnd', 'Sum1', 'Sum2', 'Sum3', 'P', 'Wn', 'Wc', 'Wdist', 'x_op', 'u_op', 'A', 'B', 'C', 'm', 'Ix', 'Iy', 'Iz', 'g', 'm_nom', 'sensor_noise_weights', 'input_disturbance', 'l_nom', 'lx', 'ly', 'tau', 'gamma', 'Wact_del', 'dkinfo');

%% Plot

% state = [x y z xd yd zd phi th psi phid thd psid]
% control = [u1 u2 u3 u4];
clear; close all; clc;

load('QuadrotorModelCLXPro/K.mat');


k.InputName = {'err'};
k.OutputName = {'u1', 'u2', 'u3', 'u4'};
% lsim simulate
t = 0:0.1:100;
num = 30;

if num ~= 0
    Parray = usample(P, num);
end
Pnom = P.nom;
yks = zeros(length(t), 16, num+1);
x_des = [0.5; 0.5; 0.5; 0; 0; 0; 0; 0; pi/12; 0; 0; 0]';
for i = 1:num
    cl = connect(Parray(:, :, i), Wn, Wdist, Wact_del, k, Sum1, Sum2, Sum3, {'dist', 'noise', 'dx_des'}, {'dxo', 'dyo', 'dzo', 'dxdo', 'dydo', 'dzdo', 'dphio', 'dtho', 'dpsio', 'dphido', 'dthdo', 'dpsido', 'u1', 'u2', 'u3', 'u4'});
    U = normrnd(0, 1, length(t), 16)./10; 
    U = [U, repmat(x_des, length(t), 1)];
    % shift by operating point
    yks(:, :, i) = lsim(cl, U, t) + [x_op', u_op'];
end
% nominal model and no noise
cl = connect(Pnom, Wn, Wdist, Wact_del, k, Sum1, Sum2, Sum3, {'dist', 'noise', 'dx_des'}, {'dxo', 'dyo', 'dzo', 'dxdo', 'dydo', 'dzdo', 'dphio', 'dtho', 'dpsio', 'dphido', 'dthdo', 'dpsido', 'u1', 'u2', 'u3', 'u4'});
U = normrnd(0, 1, length(t), 16).*0; 
U = [U, repmat(x_des, length(t), 1)];
yks(:, :, end) = lsim(cl, U, t) + [x_op', u_op'];


% plot 3D
figure; hold on;
for i = 1:num
    samp = plot3(yks(:, 1, i), yks(:, 2, i), yks(:, 3, i), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1);
end
nom = plot3(yks(:, 1, end), yks(:, 2, end), yks(:, 3, end), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
grid on; box on; axis equal;
xlabel('$$X\ [m]$$', 'interpreter', 'latex');
ylabel('$$Y\ [m]$$', 'interpreter', 'latex');
zlabel('$$Z\ [m]$$', 'interpreter', 'latex');
if num ~= 0
    legend([nom, samp], 'Nominal', 'Monte Carlo Samples');
else
    legend(nom, 'Nominal');
end

% plot control inputs
choose = 1;
figure;
subplot(4,1,1); 
plot(t, yks(:, 13, choose));
ylabel('$$U_1$$', 'interpreter', 'latex');
grid on; box on;
subplot(4,1,2); 
plot(t, yks(:, 14, choose));
grid on; box on;
ylabel('$$U_2$$', 'interpreter', 'latex');
subplot(4,1,3); 
plot(t, yks(:, 15, choose));
grid on; box on;
ylabel('$$U_3$$', 'interpreter', 'latex');
subplot(4,1,4); 
plot(t, yks(:, 16, choose));
grid on; box on;
ylabel('$$U_4$$', 'interpreter', 'latex');

% plot states
figure;
for choose = 1:num
    subplot(3,2,1); hold on;
    plot(t, yks(:, 1, choose), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1);
    plot([t(1), t(end)], [x_des(1), x_des(1)], 'k-');
    grid on; box on;
    ylabel('$$X\ [m]$$', 'interpreter', 'latex');
    subplot(3,2,3); hold on;
    plot(t, yks(:, 2, choose), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1);
    plot([t(1), t(end)], [x_des(2), x_des(2)], 'k-');
    grid on; box on;
    ylabel('$$Y\ [m]$$', 'interpreter', 'latex');
    subplot(3,2,5); hold on;
    plot(t, yks(:, 3, choose), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1);
    plot([t(1), t(end)], [x_des(3), x_des(3)], 'k-');
    grid on; box on;
    ylabel('$$Z\ [m]$$', 'interpreter', 'latex');
    xlabel('$$Time\ [s]$$', 'interpreter', 'latex');

    subplot(3,2,2); hold on;
    sample = plot(t, rad2deg(yks(:, 7, choose)), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1); 
    target = plot([t(1), t(end)], rad2deg([x_des(7), x_des(7)]), 'k-');
    grid on; box on;
    ylabel('$$\phi\ [^\circ]$$', 'interpreter', 'latex');
    subplot(3,2,4); hold on;
    plot(t, rad2deg(yks(:, 8, choose)), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1);
    plot([t(1), t(end)], rad2deg([x_des(8), x_des(8)]), 'k-');
    grid on; box on;
    ylabel('$$\theta\ [^\circ]$$', 'interpreter', 'latex');
    subplot(3,2,6); hold on;
    plot(t, rad2deg(yks(:, 9, choose)), '-', 'Color',  [0    0.4470    0.7410], 'LineWidth', 1);
    plot([t(1), t(end)], rad2deg([x_des(9), x_des(9)]), 'k-');
    grid on; box on;
    ylabel('$$\psi\ [^\circ]$$', 'interpreter', 'latex');
    xlabel('$$Time\ [s]$$', 'interpreter', 'latex');
end
% plot nominal 
subplot(3,2,1); hold on;
plot(t, yks(:, 1, end), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
plot([t(1), t(end)], [x_des(1), x_des(1)], 'k-');
grid on; box on;
    ylabel('$$X\ [m]$$', 'interpreter', 'latex');

subplot(3,2,3); hold on;
plot(t, yks(:, 2, end), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
plot([t(1), t(end)], [x_des(2), x_des(2)], 'k-');
grid on; box on;
    ylabel('$$Y\ [m]$$', 'interpreter', 'latex');

subplot(3,2,5); hold on;
plot(t, yks(:, 3, end), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
plot([t(1), t(end)], [x_des(3), x_des(3)], 'k-');
grid on; box on;
    ylabel('$$Z\ [m]$$', 'interpreter', 'latex');
    xlabel('$$Time\ [s]$$', 'interpreter', 'latex');

subplot(3,2,2); hold on;
nom = plot(t, rad2deg(yks(:, 7, end)), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
target = plot([t(1), t(end)], rad2deg([x_des(7), x_des(7)]), 'k-');
if num ~= 0
    legend([nom, sample, target], 'Nominal', 'Monte Carlo Samples', 'Target');
else
    legend([nom, target], 'Nominal', 'Target');
end
grid on; box on;
    ylabel('$$\phi\ [^\circ]$$', 'interpreter', 'latex');

subplot(3,2,4); hold on;
plot(t, rad2deg(yks(:, 8, end)), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
target = plot([t(1), t(end)], rad2deg([x_des(8), x_des(8)]), 'k-');
grid on; box on;
    ylabel('$$\theta\ [^\circ]$$', 'interpreter', 'latex');

subplot(3,2,6); hold on;
plot(t, rad2deg(yks(:, 9, end)), '-', 'Color',  [0.8500    0.3250    0.0980], 'LineWidth', 3);
target = plot([t(1), t(end)], rad2deg([x_des(9), x_des(9)]), 'k-');
grid on; box on;
    ylabel('$$\psi\ [^\circ]$$', 'interpreter', 'latex');
    xlabel('$$Time\ [s]$$', 'interpreter', 'latex');

% plot bounds
figure;
opts = bodeoptions('cstprefs');
opts.Grid = 'on';
opts.MagUnits = 'abs';
bodemag(dkinfo{1}.MussvBnds(1,1),opts);
grid on; box on;
title('Structured Singular Value');




