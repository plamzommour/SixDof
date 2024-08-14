%% Scipt to Open Data from 6DoF

clearvars;
clc;
close all;

a = importdata("MyData.txt");

% Euler Angles in Deg
figure()
subplot(2,3,1)
plot(a.data(:,1), a.data(:,5)*(180/pi), 'LineWidth', 2)
title("Phi - Roll - Degrees")
grid on
grid minor
subplot(2,3,2)
plot(a.data(:,1), a.data(:,6)*(180/pi), 'LineWidth', 2)
title("Theta - Pitch - Degrees")
grid on
grid minor
subplot(2,3,3)
plot(a.data(:,1), (a.data(:,7)*(180/pi)), 'LineWidth', 2)
title("Psi - Yaw - Degrees")
grid on
grid minor

% NED Position in Meters
subplot(2,3,4)
plot(a.data(:,1), a.data(:,2), 'LineWidth', 2)
title("X NED Position - Meters")
grid on
grid minor
subplot(2,3,5)
plot(a.data(:,1), a.data(:,3), 'LineWidth', 2)
title("Y NED Position - Meters")
grid on
grid minor
subplot(2,3,6)
plot(a.data(:,1), -a.data(:,4), 'LineWidth', 2)
title("Z NED Position - Meters - Altitude")
grid on
grid minor

% Translational Rates - Body - in Deg/Sec
figure()
subplot(3,1,1)
plot(a.data(:,1), a.data(:,8), 'LineWidth', 2)
title("U - Body - M/S")
grid on
grid minor
subplot(3,1,2)
plot(a.data(:,1), a.data(:,9), 'LineWidth', 2)
title("V - Body - M/S")
grid on
grid minor
subplot(3,1,3)
plot(a.data(:,1), a.data(:,10), 'LineWidth', 2)
title("W - Body - M/S")
grid on
grid minor

%Alpha
figure()
plot(a.data(:,1), a.data(:,11) * (180/pi), 'LineWidth', 2)
title("Alpha - Deg")
grid on
grid minor
##ylim([-90 90])

% NED Velocity in Meters/Sec
figure()
subplot(3,1,1)
plot(a.data(:,1), a.data(:,14), 'LineWidth', 2)
title("X NED Vel - Meters/Sec")
grid on
grid minor
subplot(3,1,2)
plot(a.data(:,1), a.data(:,15), 'LineWidth', 2)
title("Y NED Vel  - Meters/Sec")
grid on
grid minor
subplot(3,1,3)
plot(a.data(:,1), a.data(:,16), 'LineWidth', 2)
title("Z NED Vel - Meters/Sec")
grid on
grid minor

%Drag and W-L (+ = More Weight Contributing)
figure()
subplot(2,1,1)
plot(a.data(:,1), a.data(:,17), 'LineWidth', 2)
title("Drag Force - N - Body X")
grid on
grid minor
subplot(2,1,2)
plot(a.data(:,1), a.data(:,18), 'LineWidth', 2)
title("Lift Force - N - Body Z")
grid on
grid minor

% 3d Plot of Trajectory
figure()
plot3(a.data(:,3), a.data(:,2), -a.data(:,4), 'LineWidth', 2)
grid on
grid minor
title("Trajectory Plot - NED")
ylabel('Downtrack - X - Meters NED')
xlabel('Crosstrack - Y - Meters NED')
view(2)
axis equal

% Bank Command and Heading Error
figure()
subplot(2,1,1)
plot(a.data(:,1), a.data(:,19).*180/pi, 'LineWidth', 2)
title("Bank Command - Degrees")
grid on
grid minor
subplot(2,1,2)
plot(a.data(:,1), a.data(:,20), 'LineWidth', 2)

##% Calculate Actual Heading Error
##actual_heading = a.data(:,7); % psi
##guidepoint = [10000, 20000];
##y_pos = a.data(:,3);
##x_pos = a.data(:,2);
##del_y = guidepoint(2)-y_pos;
##del_x = guidepoint(1)-x_pos;
##bearing_2_gp = atan(del_x./del_y);
##
##heading_err = bearing_2_gp-actual_heading;
##hold on
##plot(a.data(:,1), heading_err * (180/pi))
title("Distance to Waypoint")
grid on
grid minor
