clc;
clear all
close all
data1 = load('openangleSaveWindows2018-4-24_14-49-16.TXT');
acc_close_x = data1(:,4);
acc_close_y = data1(:,5);
acc_close_z = data1(:,6);
gyro_close_x = data1(:,7);
normed_gyro_close_x = (gyro_close_x-min(gyro_close_x))/(max(gyro_close_x)-min(gyro_close_x));
gyro_close_y = data1(:,8);
gyro_close_z = data1(:,9);
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
% plot(t,Roll_close_x,'r');
% hold on;plot(t,Pitch_close_y,'b');
% hold on;
plot(t,normed_Yaw_close_z,'k');
grid on;
legend('Roll closeX','Pitch closeY','Yaw closeZ')
title('accZ RPY');
Yaw_open_max = max(Yaw_close_z)-min(Yaw_close_z)
% figure
plot(t,normed_gyro_close_x,'r');
hold on;plot(t,gyro_close_y,'b');
hold on;
plot(t,gyro_close_z,'k');
legend('gyro close x','gyro close y','gyro close z')