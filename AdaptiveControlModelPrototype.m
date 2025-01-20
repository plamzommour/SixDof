% Script to Test out some Dynamics

clearvars
clc
close all

% Start by defining some parameters

% Rocket 7 inches in diameter, 10 feet long, 500 lb

mass = 226.796; %kg

radius = 0.0889; %m

inertia = 176.785;

thrust = 5000.3886; % Force

% Define Dynamics;

drag_area = pi*radius^2;

drag_coeff = 0.5;
lift_coeff = 0.3;
cm_del = 0.3;
lever_arm = 0.0889;
launch_angle = 65;

time = 0:0.001:200;

theta_dot(1) = 0;
theta(1) = launch_angle * (pi/180);
x(1) = 0;
y(1) = 0;
x_dot(1) = launch_angle * cos(theta(1));
y_dot(1) = launch_angle * sin(theta(1));
theta_ddot(1) = 0;
del_t = 0;

for i = 2:length(time)

  rho = 1.0;

  if i > length(time)/4
    thrust = 0;
  endif

  drag  = 0.5 * rho * (sqrt(x_dot(i-1)^2 + y_dot(i-1)^2))^2 * drag_coeff; % Fake as hell
  lift = 0.5 * rho * (sqrt(x_dot(i-1)^2 + y_dot(i-1)^2))^2 * lift_coeff; % Fake as hell
  x_ddot(i) = (thrust*cos(theta(i-1)) + lift*cos(theta(i-1)) - drag*cos(theta(i-1))) / mass;
  y_ddot(i) = ((thrust*sin(theta(i-1)) + lift*sin(theta(i-1)) - drag*sin(theta(i-1))) / mass ) - 9.81;

  x_dot(i) = x_dot(i-1) + x_ddot(i).*0.001;
  x(i) = x(i-1) + x_dot(i).*0.001;
  y_dot(i) = y_dot(i-1) + y_ddot(i).*0.001;
  y(i) = y(i-1) + y_dot(i).*0.001;
  theta(i) = atan(y_dot(i)/x_dot(i)); % Velocity-Based approximation to assume perfection

  if (y_dot(i) < -10)
    return;
  endif

end

% Intention is to embed this 3-dof into a launch-phase algorithm that is a combined-guidance-control algorithm.
% Whatever the state after launcher exit is what the first state of the 3DoF should be. Estmations of
% aerodynamic forces acting on rigid body should be available.
% Comparing this to what the flight software is predicting is what creates the error signal for control action.
% Adaptive control methodology should be applied to ensure that the whole system does not go haywire and can
% correct for off-nominal perturbations
% 3DoF should be test-matched to ensure accuracy.

% If done correctly, system should be very stable and should render PIDs useless
% If boost phase, lateral guidance should be active and should be only allowed once significant capability
% is sensed.  Keep it straight until told otherwise.



