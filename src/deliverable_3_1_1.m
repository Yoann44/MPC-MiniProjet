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

x0 = [0;0;0;2];
% Get control inputs with

sys_d = c2d(sys_x, Ts);
[A,B,~,~] = ssdata(sys_d);

t_sim = 6;
t = 0:Ts:t_sim;

x(:,1) = x0;
for i = 1:length(t)-1
    u(:, i) = mpc_x.get_u(x(:,i));
    x(:,i+1) = A * x(:,i) + B * u(:, i);
end

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