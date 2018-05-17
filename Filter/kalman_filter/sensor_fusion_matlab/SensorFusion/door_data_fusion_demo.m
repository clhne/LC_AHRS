function door_data_fusion_demo
clc;
clear all;
close all;
data = load('data/prev_gyrocur_gyro prev_acc cur_acc dt2018_5_14_15-50-49.TXT');
z = data(:,2)';
prev_acc = data(:,3)';
dt = data(:,5)';
R_sensor = 0.00125;
A = 1;
C = 1;
R = 0.0125;
Q = 0.00252;
%x = 0;
x = prev_acc .* (dt/R_sensor);

xhat = kalman(z, A, C, R, Q);

clf
plotsigs(1, z, xhat, 'sensor 1')

plotsigsrms('one sensor', 2, x, xhat)

function plotsigs(pos, sig1, sig2, sig1label)
subplot(2,1,pos)
hold on
plot(sig1, 'k')
plot(sig2, 'r')
legend({sig1label, 'Estimated'})
hold off

function plotsigsrms(label, pos, x, xhat)
plotsigs(pos, x, xhat, 'Actual')
title(sprintf('%s: RMS error = %f', label, sqrt(sum((x-xhat).^2)/length(x))))