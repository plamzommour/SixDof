%% This is an example of Pro-Nav in 2D
% Written by Paul Zimmer in Dec 2024
% GOAL: Show that Pro-Nav Works by Comparing Relative Distances and
% Closing Velocities to attain an acceleration that can be sent to a
% missile control system or whatever.
% Setup Initial Conditions
clearvars; close all; clc;

% Need Initial Position, Velocities and Heading of Target and Persuer
pos_targ = [6000, 7000]; % meters
pos_purs = [1000, 0]; % meters
vel_targ = [100, 100]; % m/s
speed_purs = 300; % m/s - to be component-fied later
vel_purs = [300, 0]; % m/s, to be overwritten
head_targ = 45 * (pi/180); % rad from north
head_purs = 0 * (pi/180); % rad from north

% Need to set up integrator parameters
dt = 1/100; % 100 hz time step
time_end = 100; % end time

% Pro-Nav Parameter
N = 3;

% Prefill data
targ_trajectory = [];
purs_trajectory = [];
los_rate_plot = [];

% Enter runtime integration routine - Eulers method
for t = 0:dt:time_end % timer

    % Update target position
    pos_targ = pos_targ + vel_targ * dt;
    targ_trajectory = [targ_trajectory; pos_targ];

    % Compute relative position of Target WRT Pursuer Observation
    % ** Position of Target as Seen from Pursuer **
    rel_pos = pos_targ - pos_purs;
    % Compute Distance for Stopping
    dists = norm(rel_pos);

    % Stop if less than a meter
    if dists < 1
         break
    end

    % Find line of sight angle and estimate rate
    % los angle = atan(y_rel, x_rel);
    los_angle = atan2(rel_pos(2), rel_pos(1));
    % LOS Rate using finite difference with error trap
    if t > 0
        los_rate = (los_angle - prev_los_angle)/dt;
    else
        los_rate = 0; % First cycle has throwaway value
    end

    % Update old to calculated angle
    prev_los_angle = los_angle;
    % Calculate Closing Velocity
    r_hat = rel_pos/norm(rel_pos);
    v_close = dot(-(vel_targ - vel_purs),r_hat);

    % Establish Pro-nav law and update heading
    % Goal - Calculate acceleration needed. Drive system to zero by driving line
    % of sight rate to zero.  A_need is the acceleration needed by a
    % maneuver to put the system on a collision course.  Collision course
    % is established once LOS rate converges to zero.
    a_need = N * los_rate * v_close;

    los_rate_plot = [los_rate_plot; los_rate];

    % Update heading of chaser
    head_purs = head_purs + (a_need/v_close) * dt;

    % Update chaser position
    vel_purs = speed_purs * [cos(head_purs), sin(head_purs)];
    pos_purs = pos_purs + vel_purs * dt;
    purs_trajectory = [purs_trajectory;pos_purs];

end

% Plotting
subplot(2,1,1)
title('Target and Pursuer Trajectories - ProNav Demonstration')
plot(targ_trajectory(:,1), targ_trajectory(:,2), 'LineWidth', 2);
hold on
plot(purs_trajectory(:,1), purs_trajectory(:,2), 'LineWidth', 2);
xlabel("Crosstrack Distance")
ylabel("Downtrack Distance")
legend("Target", "Pursuer")
grid on
grid minor
axis equal
subplot(2,1,2)
plot(los_rate_plot, "LineWidth",2)
ylabel("LOS Rate")
grid on
grid minor
