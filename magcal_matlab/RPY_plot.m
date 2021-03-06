clear all;
close all;
clc;
% RPY plot
RPY = load('Roll-no mag2018_4_17_13-56-41.TXT');
Roll =  RPY(:,1);
Pitch = RPY(:,2);
Yaw =   RPY(:,3);
axis([1 10000 -180 360]);
t = 1:length(Roll);
set(gca,'Ytick',[-180 5 360]);
plot(t, Roll,'b');
hold on;
plot(t, Pitch,'r')
hold on;
plot(t, Yaw,'k');
title('RPY');
grid on