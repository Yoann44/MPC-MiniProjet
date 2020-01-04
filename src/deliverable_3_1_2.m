clear all
close all
clc

Ts       = 1/5;
quad     = Quad(Ts);
[xs, us] = quad.trim();
sys      = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

% Design MPC controller
mpc_x   = MPC_Control_x(sys_x, Ts);
mpc_y   = MPC_Control_y(sys_y, Ts);
mpc_z   = MPC_Control_z(sys_z, Ts);
mpc_yaw   = MPC_Control_yaw(sys_yaw, Ts);

t_sim = 6;
t = 0:Ts:t_sim;

x0 = [0;0;0;2];
y0 = [0;0;0;2];
z0 = [0;2];
yaw0 = [0;45/180*pi];
% Get control inputs with

[x_x, u_x] = simulate(sys_x, mpc_x, x0, t, Ts);
plotSimulationXY(x_x, u_x, t);

[x_y, u_y] = simulate(sys_y, mpc_y, y0, t, Ts);
plotSimulationXY(x_y, u_y, t);

[x_z, u_z] = simulate(sys_z, mpc_z, z0, t, Ts);
plotSimulationZ(x_z, u_z, t);

[x_yaw, u_yaw] = simulate(sys_yaw, mpc_yaw, yaw0, t, Ts);
plotSimulationYaw(x_yaw, u_yaw, t);

function [x, u] = simulate(sys, ctrl, x0, t, Ts)
    sys_d = c2d(sys, Ts);
    [A,B,~,~] = ssdata(sys_d);
    
    x(:,1) = x0;
    for i = 1:length(t)-1
        u(:, i) = ctrl.get_u(x(:,i));
        x(:,i+1) = A * x(:,i) + B * u(:, i);
    end
end

function [] = plotSimulationXY(x, u, t)
    figure;
    subplot(4, 1, 1);
    plot(t, x(4,:));
    legend("Position [m]")
    xlabel("Time [s]")
    subplot(4, 1, 2);
    plot(t, x(3,:));
    legend("Velocity [m/s]")
    xlabel("Time [s]")
    subplot(4, 1, 3);
    plot(t, x(2,:));
    legend("Angle [rad]")
    xlabel("Time [s]")
    subplot(4, 1, 4);
    plot(t, x(1,:));
    legend("Angular rate [rad/s]")
    xlabel("Time [s]")
end

function [] = plotSimulationZ(x, u, t)
    figure;
    subplot(3, 1, 1);
    plot(t, x(2,:));
    legend("Position [m]")
    xlabel("Time [s]")
    subplot(3, 1, 2);
    plot(t, x(1,:));
    legend("Velocity [m/s]")
    xlabel("Time [s]")
    subplot(3, 1, 3);
    plot(t(1:end-1), u);
    legend("Command")
    xlabel("Time [s]")
end

function [] = plotSimulationYaw(x, u, t)
    figure;
    subplot(3, 1, 1);
    plot(t, x(2,:));
    legend("Heading [rad]")
    xlabel("Time [s]")
    subplot(3, 1, 2);
    plot(t, x(1,:));
    legend("Angular Rate [rad/s]")
    xlabel("Time [s]")
    subplot(3, 1, 3);
    plot(t(1:end-1), u);
    legend("Command")
    xlabel("Time [s]")
end