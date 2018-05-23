clc;
clear all
close all
data = load('quickcloseSaveWindows2018-4-24_15-02-31.TXT');
acc_xmin = min(data(:,4));
acc_xmax = max(data(:,4));
acc_ymin = min(data(:,5));
acc_ymax = max(data(:,5));
acc_zmin = min(data(:,6));
acc_zmax = max(data(:,6));

gyro_xmin = min(data(:,7));
gyro_xmax = max(data(:,7));
gyro_ymin = min(data(:,8));
gyro_ymax = max(data(:,8));
gyro_zmin = min(data(:,9));
gyro_zmax = max(data(:,9));
fp = fopen('result.txt','w+');
fprintf(fp,...
'acc_xmin  = %f\tacc_xmax  = %f\t\nacc_ymin  = %f\tacc_ymax  = %f\t\nacc_zmin  = %f\tacc_zmax =  %f\t\ngyro_xmin = %f\tgyro_xmax = %f\t\ngyro_ymin = %f\tgyro_ymax = %f\t\ngyro_zmin = %f\tgyro_zmax = %f\t\n',...
acc_xmin,acc_xmax,acc_ymin,acc_ymax,acc_zmin,acc_zmax,gyro_xmin,gyro_xmax,gyro_ymin,gyro_ymax,gyro_zmin,gyro_zmax);
fclose(fp);
t = 1:length(data(:,4));
plot(t,data(:,4),'r');
hold on;plot(t,data(:,5),'b');
hold on;plot(t,data(:,6),'g');
legend('accX','accY','accZ')
figure;
plot(t,data(:,7)*180/3.14,'c');
hold on;plot(t,data(:,8)*180/3.14,'m');
hold on;plot(t,data(:,9)*180/3.14,'k');
legend('gyroX','gyroY','gyroZ');