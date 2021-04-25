% Clear variables, close figures and refresh command line
close all
clear all
clc

%% Create the input for parameter estimation
sim('init_input.slx');

%% Initialize parameters
motor_r = 1;
motor_l = 0.6;
motor_k = 0.002;
motor_j = 10;
motor_b = 0.002;

%% Import experiment data
experiment_data = importdata('experiment_giltay.csv');

%% Interpolate data so that vectors are equally long
time = 0:0.01:4;
time = time';
rpm = experiment_data(4:end,2);

x01 = 1:numel(time);
x02 = 1:numel(rpm);
xi2 = linspace(1,numel(x02),401);

rpm_correct = interp1(x02,rpm,xi2)';

%% Plot
plot(input, 'linewidth',2)
hold on
grid on
plot(time,rpm_correct, 'linewidth',2)
legend('Input', 'Output');

