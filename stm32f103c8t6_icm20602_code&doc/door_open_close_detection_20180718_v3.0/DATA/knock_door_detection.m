clc;clear all;close all;
dump_file='knock_door1119.txt';
data = load(dump_file);
ts = data(:,1); is_gyro_dyn = data(:,2); is_gyro_calib = data(:,3); is_acc_dyn = data(:,4);
is_acc_calib = data(:,5); roll = data(:,6); pitch = data(:,7); yaw = data(:,8); 
raw_ax = data(:,9);raw_ay = data(:,10);raw_az = data(:,11);filt_ax = data(:,12);
filt_ay = data(:,13);filt_az = data(:,14);raw_gx = data(:,15);raw_gy = data(:,16);
raw_gz = data(:,17);cor_gx = data(:,18);cor_gy = data(:,19);cor_gz = data(:,20); 
figure;
plot(ts, roll, 'r');hold on;
plot(ts, pitch, 'g');hold on;
plot(ts, yaw, 'b');legend('roll','pitch', 'yaw');
grid on
figure;
plot(ts, raw_ax, 'r');hold on;
plot(ts, raw_ay, 'g');hold on;
plot(ts, raw_az, 'b');hold on;
plot(ts, filt_ax, 'y');hold on;
plot(ts, filt_ay, 'k');hold on;
plot(ts, filt_az, 'c');hold on;
legend('rawax','raway', 'rawaz', 'filtax','filtay', 'filtaz');
grid on
figure;
plot(ts, raw_gx, 'r');hold on;
plot(ts, raw_gy, 'g');hold on;
plot(ts, raw_gz, 'b');hold on;
plot(ts, cor_gx, 'y');hold on;
plot(ts, cor_gy, 'k');hold on;
plot(ts, cor_gz, 'c');
legend('rawgx','rawgy', 'rawgz', 'corgx','corgy', 'corgz');
grid on



