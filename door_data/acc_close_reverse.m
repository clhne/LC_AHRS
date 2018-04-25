clc;
clear all
close all
data1 = load('accClosedreverseSaveWindows2018-4-24_14-52-10.TXT');
acc_close_x = data1(:,4);
acc_close_y = data1(:,5);
acc_close_z = data1(:,6);
normed_acc_close_z = (acc_close_z-min(acc_close_z))/(max(acc_close_z)-min(acc_close_z));
Roll_close_x = data1(:,1);
Pitch_close_y = data1(:,2);
Yaw_close_z = data1(:,3);
normed_Yaw_close_z = (Yaw_close_z-min(Yaw_close_z))/(max(Yaw_close_z)-min(Yaw_close_z));
t = 1:length(acc_close_x);
% plot(t,acc_close_x,'r');
% hold on;plot(t,acc_close_y,'b');
hold on;plot(t,acc_close_z,'g');
% legend('accX','accY','accZ');
% figure
% plot(t,angle_close_x,'r');
% hold on;plot(t,angle_close_y,'b');
% hold on;
plot(t,Yaw_close_z,'k');
grid on;
legend('accZ','Yaw')
title('accZ RPY');

