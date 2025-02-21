% Inverted Pendulum Simulation in Octave with LQR Control

% System Parameters
m = 0.2;    % mass of the pendulum (kg)
M = 0.5;    % mass of the cart (kg)
l = 0.3;    % length to pendulum center of mass (m)
g = 9.81;   % gravity (m/s^2)

% State-Space Representation
A = [0 1 0 0;
     0 0 -(m*g/M) 0;
     0 0 0 1;
     0 0 ((M+m)*g/(M*l)) 0];

B = [0;
     1/M;
     0;
     -1/(M*l)];

C = eye(4);
D = zeros(4,1);

% Manual LQR Implementation in Octave
% Solve Riccati equation manually since Octave does not have lqr()
% P is obtained by solving continuous-time algebraic Riccati equation
Q = diag([10, 1, 10, 1]);
R = 0.01;
[P, ~, ~] = care(A, B, Q, R);
K = inv(R) * B' * P;   % LQR gain

% Initial Conditions
x0 = [0.1; 0; 0.1; 0];  % Initial state [cart_pos; cart_vel; pend_angle; pend_ang_vel]

% Simulation Setup
ts = 0.01;               % Sampling time
T = 10;                  % Total simulation time
N = T/ts;                % Number of time steps

x = x0;
x_history = zeros(4, N);
u_history = zeros(1, N);

t = 0:ts:T-ts;

for k = 1:N
    u = -K * x;           % LQR control law
    x_dot = A*x + B*u;    % State derivatives
    x = x + x_dot * ts;   % State update using Euler method
    x_history(:, k) = x;
    u_history(k) = u;
end

% Plotting Results
figure;
subplot(2,1,1);
plot(t, x_history(1,:), t, x_history(3,:));
title('Cart Position and Pendulum Angle');
xlabel('Time (s)');
ylabel('State');
legend('Cart Position (m)', 'Pendulum Angle (rad)');

title('Inverted Pendulum Simulation with LQR Control');

subplot(2,1,2);
plot(t, u_history);
title('Control Input');
xlabel('Time (s)');
ylabel('Force (N)');

