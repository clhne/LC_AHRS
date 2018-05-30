clc;clear all;close all;
%dump_file='open_close_door_fast1.txt';
dump_file='open_close_door_fast2.txt';
%dump_file='open_close_door_fast3.txt';
%dump_file='open_close_door_normal1.txt';
%dump_file='open_close_door_normal2.txt';
%dump_file='open_close_door_slow1.txt';
%dump_file='open_close_door_slow2.txt';
%dump_file='open_close_door_failed1.txt';
%dump_file='open_close_door_failed2.txt';
%dump_file='open_close_door_success1.txt';
%dump_file='open_close_door_success2.txt';
%dump_file='open_close_door_success3.txt';
%dump_file='open_close_door_success4.txt';
%dump_file='open_close_door_success5.txt';
%dump_file='open_close_door_success6.txt';
%dump_file='open_close_door_success7_quiet.txt';
%dump_file='open_close_door_failed3.txt';
%dump_file='open_close_door_failed_to_sucess1.txt';
%dump_file='open_close_door_failed_to_sucess2.txt';
%dump_file='SaveWindows2018-5-29_10-58-47.txt';
% [ts, is_gyro_dyn, is_gyro_calib, is_acc_dyn, is_acc_calib, roll, pitch, yaw, raw_ax, raw_ay, raw_az,... 
%     filt_ax, filt_ay, filt_az, raw_gx, raw_gy, raw_gz, cor_gx, cor_gy, cor_gz] = textread(dump_file,...
% '%d %d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n');
data = load(dump_file);
ts = data(:,1); is_gyro_dyn = data(:,2); is_gyro_calib = data(:,3); is_acc_dyn = data(:,4);
is_acc_calib = data(:,5); roll = data(:,6); pitch = data(:,7); yaw = data(:,8); 
raw_ax = data(:,9);raw_ay = data(:,10);raw_az = data(:,11);filt_ax = data(:,12);
filt_ay = data(:,13);filt_az = data(:,14);raw_gx = data(:,15);raw_gy = data(:,16);
raw_gz = data(:,17);cor_gx = data(:,18);cor_gy = data(:,19);cor_gz = data(:,20); 
%{
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
%}
figure;
plot(ts, pitch, 'r');hold on;
plot(ts, raw_az, 'g');hold on;
plot(ts, cor_gx, 'b');
% oscillation detection
N = length(ts);
count_peak = 0;
count_trough = 0;
peak_trough_init = 0;
peak_trough_index = 1;
for i = 102:N
   if(abs(pitch(i))<1.5)
       if(0 == is_gyro_dyn(i) && 0 == is_acc_dyn(i))
           break;
       end
        if(cor_gx(i-1) < cor_gx(i))
            cor_gx_peak = cor_gx(i);
            count_peak = count_peak+1;
            cur_monotonicity = 1;
            %fprintf('peak i=%d ts=%d cor_gx=%f count_peak= %d\n',i,ts(i),cor_gx(i),count_peak)
        elseif(cor_gx(i-1) > cor_gx(i))
            cor_gx_trough = cor_gx(i);
            count_trough = count_trough + 1;
            cur_monotonicity = 0;
            %fprintf('trough i=%d ts=%d cor_gx=%f count_trough= %d\n',i,ts(i),cor_gx(i),count_trough)
        else
            continue;
        end
        if(peak_trough_init == 1)
            if(cur_monotonicity ~= pre_monotonicity)
                if(cur_monotonicity == 0 && pre_monotonicity == 1 && cor_gx(i-1)>0)
                    peak_trough_ts(peak_trough_index) = ts(i-1);
                    peak_trough_value(peak_trough_index) = cor_gx(i-1);
                    flag_peak_trough(peak_trough_index) = 1;
                    peak_trough_index = peak_trough_index + 1;
                    fprintf('peak i=%d ts=%d cor_gx=%f count_peak= %d\n',i,ts(i),cor_gx(i-1),count_peak)
                elseif(cur_monotonicity == 1 && pre_monotonicity ==0 && cor_gx(i-1)<0)
                    peak_trough_ts(peak_trough_index) = ts(i-1);
                    peak_trough_value(peak_trough_index) = cor_gx(i-1);
                    flag_peak_trough(peak_trough_index) = 0;
                    peak_trough_index = peak_trough_index + 1;
                    fprintf('trough i=%d ts=%d cor_gx=%f count_trough= %d\n',i,ts(i),cor_gx(i-1),count_trough)
                end
            end
        else
            peak_trough_init = 1;
        end
          pre_monotonicity = cur_monotonicity;  
   else
       peak_trough_init = 0;
       peak_trough_index = 1;
   end
end
peak_trough_plot_first = 0;
peak_trough_plot_index = 1;
for i = 2: peak_trough_index
    if (peak_trough_plot_first == 0)
        if(abs(peak_trough_value(i-1)) > 0.1)
            peak_trough_plot_first = 1;
        else
            continue;
        end
    else
        if(peak_trough_plot_first == 1 && abs(peak_trough_value(i-1)) < 0.1 && abs(peak_trough_value(i)) < 0.1)
            break;
        end
    end
    plot(peak_trough_ts(i-1),peak_trough_value(i-1),'ko');
    peak_trough_plot_ts(peak_trough_plot_index) = peak_trough_ts(i-1);
    peak_trough_plot_value(peak_trough_plot_index) = peak_trough_value(i-1);
    peak_trough_plot_flag = flag_peak_trough(i-1);
    peak_trough_plot_index = peak_trough_plot_index + 1;
end
legend('pitch','filtaz','corgx','peak','trough');
grid on

