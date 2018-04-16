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
% mag = load('412.TXT');
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
mag = load('icm20602-finecalSaveWindows2018_4_13_16-22-56.txt');
mag_x = mag(:,1);
mag_y = mag(:,2);
mag_z = mag(:,3);
plot(mag_x, mag_y,'bo');
hold on;
plot(mag_x, mag_z,'r*');
hold on;
plot(mag_z, mag_y,'kv');
grid on
title('ICM20602 Fine cal');


% figure;
% mag = load('22.TXT');
% mag_x = mag(:,1);
% mag_y = mag(:,2);
% mag_z = mag(:,3);
% plot(mag_x, mag_y,'bo');
% hold on;
% plot(mag_x, mag_z,'r*');
% hold on;
% plot(mag_y, mag_z,'kv');
% title('magcal ICM20602');

% y = load('1.TXT');
% y_max = max(y)
% y_min = min(y)
% bias = (y_max + y_min)/2