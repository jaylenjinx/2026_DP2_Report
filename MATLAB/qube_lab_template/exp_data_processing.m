%% Use this file in case you want to plot saved data
%
% 41277 Control Design
% University of Technology Sydney, Australia
%
% A/Prof Ricardo Aguilera
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
s = tf('s'); %define Laplace operator

%% Read saved experimental data
%change the data test number accordingly
dataFile = 'data_test_01.mat';
load(dataFile)

%% Plot measured data
% use this as an example to create plots for your final report
figure(1)
subplot(311)
plot(time, Vm)
grid
ylabel('V_m [V]')

subplot(312)
plot(time,rad2deg(theta_rad))
grid
ylabel('\theta [^o]')

subplot(313)
plot(time, rad2deg(alpha_rad))
grid
ylabel('\alpha [^o]')


set(findall(gcf,'Type','Line'),'LineWidth',2) % increase linewidth

