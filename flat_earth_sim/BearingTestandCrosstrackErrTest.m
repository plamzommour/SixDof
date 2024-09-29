%% Script to Test Waypoint Bearing Calculation and Crosstrack Position Error
clearvars;
clc;
close all;

% Create some random waypoints in space (downtrack, crosstrack)
wp1 = [0, 0];
wp2 = [-20000, 5000];

% Find bearing RELATIVE TO NORTH of these numbers.
% Always use ending point as point 2
x_err = wp2(1)-wp1(1);
y_err = wp2(2)-wp1(2);

% Find Angle WRT to North - Bearing
bearing = atan2(y_err,x_err) * (180/pi) ;

% Find Current EXPECTED Crosstrack Position, Assuming a DOWNTRACK Position
downtrack_test = -50;
crosstrack_expected = downtrack_test * tand(bearing);

% Formulate 2x2 Rotation Matrix
rotz_ned = [cosd(bearing),sind(bearing);-sind(bearing),cosd(bearing)];

% Assemble Positions into Vectors
wp1_rot = [wp1];
wp2_rot = [wp2];

% Add a Random Point Close to Line
actual_position = [0, 5000];

% Rotate into Mathematically Easy Position - Straight Line Pointed Up and Down.
rotated_wp1 = rotz_ned * wp1_rot';
rotated_wp2 = rotz_ned * wp2_rot';
rotated_position = rotz_ned * actual_position';

% Formulate crosstrack error using Rotated Vector
crosstrack_error = rotated_position(2) - rotated_wp1(2);

% Formulate Bank Required to Steer Out Crosstrack error
bank_command = crosstrack_error * -0.010;

%% Show in plotting

figure()
hold on
scatter(wp1(2), wp1(1), 50, "b", 'filled');
scatter(wp2(2), wp2(1), 50, "b", 'filled');
scatter(actual_position(2), actual_position(1), 50, "r", 'filled')
plot([wp1(2), wp2(2)],[wp1(1), wp2(1)], '--', 'LineWidth', 2)
scatter(rotated_wp1(2), rotated_wp1(1), 50, "b", 'filled');
scatter(rotated_wp2(2), rotated_wp2(1), 50, "b", 'filled');
scatter(rotated_position(2), rotated_position(1), 50, "r", 'filled')
plot([rotated_wp1(2), rotated_wp2(2)],[rotated_wp1(1), rotated_wp2(1)], '--', 'LineWidth', 2)
grid on; grid minor;
axis equal;
