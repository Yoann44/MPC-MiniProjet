clear all
close all
clc

set(groot,'DefaultAxesFontSize',14);
set(groot,'DefaultLineLineWidth',1.5);

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

x_ref = 2;
y_ref = 2;
z_ref = 2;
yaw_ref = 45/180*pi;

x0 = [0;0;0;0];
y0 = [0;0;0;0];
z0 = [0;0];
yaw0 = [0;0];
% Get control inputs with

[x_x, u_x] = simulate(sys_x, mpc_x, x0, x_ref, t, Ts);
plotSimulationXY(x_x, u_x, t, "X", x_ref);

[x_y, u_y] = simulate(sys_y, mpc_y, y0, y_ref, t, Ts);
plotSimulationXY(x_y, u_y, t, "Y", y_ref);

[x_z, u_z] = simulate(sys_z, mpc_z, z0, z_ref, t, Ts);
plotSimulationZ(x_z, u_z, t, z_ref);

[x_yaw, u_yaw] = simulate(sys_yaw, mpc_yaw, yaw0, yaw_ref, t, Ts);
plotSimulationYaw(x_yaw, u_yaw, t, yaw_ref);

function [x, u] = simulate(sys, ctrl, x0, ref, t, Ts)
    sys_d = c2d(sys, Ts);
    [A,B,~,~] = ssdata(sys_d);
    
    x(:,1) = x0;
    for i = 1:length(t)-1
        u(:, i) = ctrl.get_u(x(:,i), ref);
        x(:,i+1) = A * x(:,i) + B * u(:, i);
    end
end

function [] = plotSimulationXY(x, u, t, axe, ref)
    figure;
    
    sgtitle(sprintf("Simulation of %s axis", axe))
    subplot(3, 2, [1 2]);
    plot(t, x(4,:));
    ylim([min(x(4,:))-0.2 max(x(4,:))+0.2])
    hold on;
    plot(t, ref*ones(1,length(t)), "--", "color", "#EDB120");
    legend("sim", "ref")
    ylabel("Position [m]")
    xlabel("Time [s]")
    
    subplot(3, 2, 3);
    plot(t, x(3,:));
    legend("sim")
    ylabel("Velocity [m/s]")
    xlabel("Time [s]")
    
    subplot(3, 2, 4);
    plot(t, x(2,:));
    legend("sim")
    ylabel("Angle [rad]")
    xlabel("Time [s]")
    
    subplot(3, 2, 5);
    plot(t, x(1,:));
    legend("sim")
    ylabel("Angular rate [rad/s]")
    xlabel("Time [s]")
    
    subplot(3, 2, 6);
    plot(t(1:end-1), u);
    legend("sim")
    ylabel("Command")
    xlabel("Time [s]")
end

function [] = plotSimulationZ(x, u, t, ref)
    figure;
    sgtitle("Simulation of Z axis")
    
    subplot(2, 2, [1 2]);
    plot(t, x(2,:));
    ylim([min(x(2,:))-0.2 max(x(2,:))+0.2])
    hold on;
    plot(t, ref*ones(1,length(t)), "--", "color", "#EDB120");
    legend("sim", "ref")
    ylabel("Position [m]")
    xlabel("Time [s]")
    
    subplot(2, 2, 3);
    plot(t, x(1,:));
    legend("sim")
    ylabel("Velocity [m/s]")
    xlabel("Time [s]")
    
    subplot(2, 2, 4);
    plot(t(1:end-1), u);
    legend("sim")
    ylabel("Command")
    xlabel("Time [s]")
end

function [] = plotSimulationYaw(x, u, t, ref)
    figure;
    sgtitle("Simulation of Yaw axis")
    
    subplot(2, 2, [1 2]);
    plot(t, x(2,:));
    ylim([min(x(2,:))-0.2 max(x(2,:))+0.2])
    hold on;
    plot(t, ref*ones(1,length(t)), "--", "color", "#EDB120");
    legend("sim", "ref")
    ylabel("Heading [rad]")
    xlabel("Time [s]")
    
    subplot(2, 2, 3);
    plot(t, x(1,:));
    legend("sim")
    ylabel("Angular Rate [rad/s]")
    xlabel("Time [s]")
    
    subplot(2, 2, 4);
    plot(t(1:end-1), u);
    legend("sim")
    ylabel("Command")
    xlabel("Time [s]")
end