clc;
clear all;
close all;
%dump_file='open_close_door_fast1.txt';
%dump_file='open_close_door_fast2.txt';
%dump_file='open_close_door_fast3.txt';
%dump_file='open_close_door_normal1.txt';
%dump_file='open_close_door_normal2.txt';
%dump_file='open_close_door_slow1.txt';
%dump_file='open_close_door_slow2.txt';
dump_file='SaveWindows2018-5-29_10-58-47.txt';
[ts, is_gyro_dyn, is_gyro_calib, is_acc_dyn, is_acc_calib, roll, pitch, yaw, raw_ax, raw_ay, raw_az,... 
    filt_ax, filt_ay, filt_az, raw_gx, raw_gy, raw_gz, cor_gx, cor_gy, cor_gz] = textread(dump_file,...
'%d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n');
%{
figure;
plot(ts, roll, 'r');
hold on;
plot(ts, pitch, 'g');
hold on;
plot(ts, yaw, 'b');
legend('roll','pitch', 'yaw');
grid on
figure;
plot(ts, raw_ax, 'r');
hold on;
plot(ts, raw_ay, 'g');
hold on;
plot(ts, raw_az, 'b');
hold on;
plot(ts, filt_ax, 'y');
hold on;
plot(ts, filt_ay, 'k');
hold on;
plot(ts, filt_az, 'c');
hold on;
legend('rawax','raway', 'rawaz', 'filtax','filtay', 'filtaz');
grid on
figure;
plot(ts, raw_gx, 'r');
hold on;
plot(ts, raw_gy, 'g');
hold on;
plot(ts, raw_gz, 'b');
hold on;
plot(ts, cor_gx, 'y');
hold on;
plot(ts, cor_gy, 'k');
hold on;
plot(ts, cor_gz, 'c');
legend('rawgx','rawgy', 'rawgz', 'corgx','corgy', 'corgz');
grid on
%}
figure;
plot(ts, pitch, 'r');
hold on;
plot(ts, raw_az, 'g');
hold on;
plot(ts, cor_gx, 'b');
legend('pitch','filtaz', 'corgx');
grid on
% oscillation detection
N = length(ts) - 2;
count_peak = 0;
count_trough = 0;
is_increase = 1;
is_decrease = 1;
flag_peak = 1;
flag_trough = 1;
for i = 2:N
   if(abs(pitch(i)) < abs(pitch(i-1)) && abs(pitch(i))<0.5)
        if(is_increase && flag_peak && cor_gx(i-1) < cor_gx(i))
            cor_gx_peak = cor_gx(i);
            count_peak = count_peak+1;
            is_increase = 0;
            flag_peak = 0;
            fprintf('peak i=%d ts=%d cor_gx=%f count_peak= %d\n',i,ts(i),cor_gx(i),count_peak)
        end
        if(is_decrease && flag_trough && cor_gx(i-1) > cor_gx(i))
            cor_gx_trough = cor_gx(i);
            count_trough = count_trough + 1;
            is_decrease = 0;
            flag_trough = 0;
            fprintf('trough i=%d ts=%d cor_gx=%f count_trough= %d\n',i,ts(i),cor_gx(i),count_trough)
        end
    end
end
