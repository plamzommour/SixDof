%% This script plots all the data for the bullet simulation

a = importdata('MyData.txt');

a = a.data;

% Plotting

subplot(2, 1, 1)
plot(a(:,2), a(:,3),'LineWidth', 2)
grid on
grid minor
xlabel('X Distance (Meters)')
ylabel('Y Distance (Meters)')
title('Trajectory Plot')

subplot(2, 1, 2)
plot(a(:,1), a(:,6) * (180/pi),'LineWidth', 2)
grid on
grid minor
xlabel('Time (sec)')
ylabel('Pitch (Deg)')
title('Pitch Plot')
