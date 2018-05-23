clear; close all; clc;

choice = 'sample'; % 'nom' for nominal; 'sample' for samplied
controller = 0; % 0 for mu-synthesis; 1 for LQR
num = 20;
factor = 0.35;

% desired state
% state = [x y z xd yd zd phi th psi phid thd psid]
x_des = [1; 1; 1; 0; 0; 0; 0; 0; pi/12; 0; 0; 0];

for i = 1:num
    i
    % load program parameters
    initial_conditions
    LoadQuadrotorConst_XPro1a

    % simulink model
    sim('CL_Xpro_model');

    % plot data
    figure(1);
    subplot(3,2,1); hold on;
    plot(x.Time, x.Data,'-', 'Color',  [0    0.4470    0.7410]);
    plot([x.Time(1), x.Time(end)], [x_des(1), x_des(1)], 'k-');
    grid on; box on;
    ylabel('X [m]');
    subplot(3,2,3); hold on;
    plot(y.Time, y.Data,'-', 'Color',  [0    0.4470    0.7410]);
    plot([y.Time(1), y.Time(end)], [x_des(2), x_des(2)], 'k-');
    grid on; box on;
    ylabel('Y [m]');
    subplot(3,2,5); hold on;
    plot(z.Time, z.Data,'-', 'Color',  [0    0.4470    0.7410]);
    plot([z.Time(1), z.Time(end)], [x_des(3), x_des(3)], 'k-');
    grid on; box on;
    ylabel('Z [m]');
    xlabel('Time [s]');

    subplot(3,2,2); hold on;
    plot(Phi.Time, Phi.Data,'-', 'Color',  [0    0.4470    0.7410]);
    plot([Phi.Time(1), Phi.Time(end)], [x_des(7), x_des(7)], 'k-');
    grid on; box on;
    ylabel('\phi [rad]');
    subplot(3,2,4); hold on;
    plot(Theta.Time, Theta.Data,'-', 'Color',  [0    0.4470    0.7410]);
    plot([Theta.Time(1), Theta.Time(end)], [x_des(8), x_des(8)], 'k-');
    grid on; box on;
    ylabel('\theta [rad]');
    subplot(3,2,6); hold on;
    plot(Psi.Time, Psi.Data,'-', 'Color',  [0    0.4470    0.7410]);
    plot([Psi.Time(1), Psi.Time(end)], [x_des(9), x_des(9)], 'k-');
    grid on; box on;
    ylabel('\Psi [rad]');
    xlabel('Time [s]');

    figure(2); hold on;
    plot3(x.Data, y.Data, z.Data,'-', 'Color',  [0    0.4470    0.7410]);
    grid on; box on; axis equal;
    xlabel('X m');
    ylabel('Y m');
    zlabel('Z m');

    figure(3);
    subplot(4,1,1);
    plot(U.Time, U.Data(:, 1))
    grid on; box on;
    ylabel('U_1');

    subplot(4,1,2);
    plot(U.Time, U.Data(:, 2))
    grid on; box on;
    ylabel('U_2');

    subplot(4,1,3);
    plot(U.Time, U.Data(:, 3))
    grid on; box on;
    ylabel('U_3');

    subplot(4,1,4);
    plot(U.Time, U.Data(:, 4))
    grid on; box on;
    ylabel('U_4');
end
