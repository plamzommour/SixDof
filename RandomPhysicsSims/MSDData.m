%% This script plots all the data for the bullet simulation

a = importdata('MyData.txt');

a = a.data;

% Plotting

plot(a(:,2), a(:,3),'LineWidth', 3)
grid on
grid minor
xlabel('Time [sec]')
ylabel('Y Distance (Meters)')
title('Trajectory Plot - Bouncing Ball')
