%% This script plots all the data for the bullet simulation

a = importdata('MyData.csv');

a = a.data;

% Plotting
hold on

plot(a(:,2), a(:,3),'LineWidth', 3)
grid on
grid minor
xlabel('X Distance (Meters)')
ylabel('Y Distance (Meters)')
title('Trajectory Plot - Projectile')
