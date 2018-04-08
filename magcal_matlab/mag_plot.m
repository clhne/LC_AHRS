clear all;
close all;
clc;
% LC1860_yangjinchuan mag x-y,x-z,y-z plot
% 
% mag = load('Mag_data_04041058_yangjingchuan.txt');
% mag_x = mag(:,1);
% mag_y = mag(:,2);
% mag_z = mag(:,3);
% plot(mag_x, mag_y,'bo');
% hold on;
% plot(mag_x, mag_z,'r*');
% hold on;
% plot(mag_y, mag_z,'kv');
% title('LC1860-yangjinchuan');

% LC1860_mine mag x-y,x-z,y-z plot
% figure;
% mag = load('Mag_data_04041113mine.txt');
% mag_x = mag(:,1);
% mag_y = mag(:,2);
% mag_z = mag(:,3);
% plot(mag_x, mag_y,'bo');
% hold on;
% plot(mag_x, mag_z,'r*');
% hold on;
% plot(mag_y, mag_z,'kv');
% title('LC1860-mine');

% MPU9250 mag x-y,x-z,y-z plot
% figure;
% mag = load('SaveWindows2018_4_8_11-50-22.TXT');
% mag_x = mag(:,1);
% mag_y = mag(:,2);
% mag_z = mag(:,3);
% plot(mag_x, mag_y,'bo');
% hold on;
% plot(mag_x, mag_z,'r*');
% hold on;
% plot(mag_y, mag_z,'kv');
% title('MPU9250');

figure;
mag = load('uncalicmSaveWindows2018_4_8_14-59-55.TXT');
mag_x = mag(:,1);
mag_y = mag(:,2);
mag_z = mag(:,3);
plot(mag_x, mag_y,'bo');
hold on;
plot(mag_x, mag_z,'r*');
hold on;
plot(mag_y, mag_z,'kv');
title('uncal ICM20602');

figure;
mag = load('icmSaveWindows2018_4_8_15-16-51.TXT');
mag_x = mag(:,1);
mag_y = mag(:,2);
mag_z = mag(:,3);
plot(mag_x, mag_y,'bo');
hold on;
plot(mag_x, mag_z,'r*');
hold on;
plot(mag_y, mag_z,'kv');
title('magcal ICM20602');