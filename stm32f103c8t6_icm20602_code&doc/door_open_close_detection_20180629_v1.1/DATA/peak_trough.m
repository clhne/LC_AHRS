clc;clear all;close all;
fprintf('--------------------------------------------------\nPeak and trough detection:\n');
%dump_file='open_close_door_fast1.txt';
%dump_file='open_close_door_fast2.txt';
%dump_file='open_close_door_fast3.txt';
%dump_file='open_close_door_normal1.txt';
%dump_file='open_close_door_normal2.txt';
%dump_file='open_close_door_slow1.txt';
%dump_file='open_close_door_slow2.txt';
%dump_file='open_close_door_success1.txt';      % 波谷不单调
%dump_file='open_close_door_success2.txt';      % 波谷不单调
%dump_file='open_close_door_success3.txt';      % 波谷不单调
%dump_file='open_close_door_success4.txt';
%dump_file='open_close_door_success5.txt';      % 波谷不单调
%dump_file='open_close_door_success6.txt';
%dump_file='open_close_door_success7_quiet.txt';
%dump_file='SaveWindows2018-5-29_10-48-17.txt';      % 波峰、波谷不单调，考虑关门时，门自身问题
%dump_file='SaveWindows2018-5-29_10-51-08.txt';      % 波谷不单调
%dump_file='SaveWindows2018-5-29_10-51-50.txt';      % 波峰、波谷不单调
%dump_file='SaveWindows2018-5-29_10-53-01.txt';      % 波峰、波谷不单调
%dump_file='SaveWindows2018-5-29_10-55-40.txt';      % 波峰、波谷不单调
%dump_file='SaveWindows2018-5-29_10-58-47.txt';       % 波峰、波谷不单调
%dump_file='SaveWindows2018_6_1_13-47-43.txt'; 
%dump_file='SaveWindows2018_6_1_13-56-26.txt';
%dump_file = 'SaveWindows2018_6_6_8-54-04.txt';
%dump_file = 'SaveWindows2018_6_6_9-20-36.txt';
dump_file = 'SaveWindows2018-6-12_16-29-08.txt';

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
plot(ts, cor_gx, 'b');hold on;
plot(ts, is_gyro_dyn, 'k');hold on;
plot(ts, is_acc_dyn, 'c');
% oscillation detection
N = length(ts);
count_peak = 0;
count_trough = 0;
peak_trough_init = 0;
peak_index = 1;
trough_index = 1;
for i = 200:N
   if(abs(pitch(i))<1.5)
       if(0 == is_gyro_dyn(i) && 0 == is_acc_dyn(i))     %i == 102
           break;
       end
        if(cor_gx(i-1) < cor_gx(i))
            cor_gx_peak = cor_gx(i);
            
            cur_monotonicity = 1;
        elseif(cor_gx(i-1) > cor_gx(i))
            cor_gx_trough = cor_gx(i);
            
            cur_monotonicity = 0;
        else
            continue;
        end
        if(peak_trough_init == 1)
            if(cur_monotonicity ~= pre_monotonicity)
                if(cur_monotonicity == 0 && pre_monotonicity == 1 && cor_gx(i-1)>0)
                    peak_ts(peak_index) = ts(i-1);
                    peak_value(peak_index) = cor_gx(i-1);
                    flag_peak(peak_index) = 1;
                    peak_index = peak_index + 1;
%                     if(peak_value(peak_index-1) < 0.062 )
%                         break;
%                     end
%                     if(peak_value(peak_index-1) == peak_value(peak_index-2))
%                         break;
%                     end
%                     if(peak_index >= 6)
%                         break;
%                     end
                    %fprintf('peak i=%d ts=%d cor_gx=%f count_peak= %d\n',i,ts(i),cor_gx(i-1),peak_index-2)
                   peak_ts;
                elseif(cur_monotonicity == 1 && pre_monotonicity ==0 && cor_gx(i-1)<0)
                    trough_ts(trough_index) = ts(i-1);
                    trough_value(trough_index) = cor_gx(i-1);
                    flag_trough(trough_index) = 0;
                    trough_index = trough_index + 1;
%                     if(abs(trough_value(trough_index-1)) < 0.062)
%                         break;
%                     end
%                     if(trough_value(trough_index-1) == trough_value(trough_index-2))
%                         break;
%                     end
%                     if(trough_index >= 6) 
%                         break;
%                     end
                    %fprintf('trough i=%d ts=%d cor_gx=%f count_trough= %d\n',i,ts(i),cor_gx(i-1),trough_index-2)
                    trough_ts;
                end
            end
        else
            peak_trough_init = 1;
        end
          pre_monotonicity = cur_monotonicity;  
   else
       peak_trough_init = 0;
       peak_index = 1;
       trough_index = 1;
   end
end
peak_plot_first = 0;
peak_plot_index = 1;
for i = 2: peak_index-1 
    if (peak_plot_first == 0)
        if(abs(peak_value(i-1)) > 0.061)
            peak_plot_first = 1;
        else
            continue;
        end
    else
        if(peak_plot_first == 1 && abs(peak_value(i-1)) < 0.061 && abs(peak_value(i)) < 0.061)
            break;
        end
    end
    if(peak_value(i) > peak_value(i-1))
        fprintf('not monotonicity i=%d,peak_value=%f\n',i,peak_value(i));
    else%if(peak_value(i-1) >= peak_value(i))
        count_peak = count_peak+1;
    end
    plot(peak_ts(i-1),peak_value(i-1),'*r');
    plot(peak_ts(i),peak_value(i),'*r');
    peak_plot_ts(peak_plot_index) = peak_ts(i-1);
    peak_plot_value(peak_plot_index) = peak_value(i-1);
    peak_plot_flag = flag_peak(i-1);
    peak_plot_index = peak_plot_index + 1;
end
trough_plot_first = 0;
trough_plot_index = 1;
for i = 2: trough_index-1 
    if (trough_plot_first == 0)
        if(abs(trough_value(i-1)) > 0.061)
            trough_plot_first = 1;
        else
            continue;
        end
    else
        if(trough_plot_first == 1 && abs(trough_value(i-1)) < 0.061 && abs(trough_value(i)) < 0.061)
            break;
        end
    end
    if(abs(trough_value(i)) > abs(trough_value(i-1)))
        fprintf('not monotonicity i=%d,trough_value=%f\n',i,trough_value(i));
    else%if(abs(trough_value(i)) <= abs(trough_value(i-1)))
        count_trough = count_trough + 1;
    end
    plot(trough_ts(i-1),trough_value(i-1),'*','color',[0.99 0.05 0.99]); 
    plot(trough_ts(i),trough_value(i),'*','color',[0.99 0.05 0.99]);     %https://zh.wikipedia.org/zh-cn/%E4%B8%89%E5%8E%9F%E8%89%B2%E5%85%89%E6%A8%A1%E5%BC%8F
    trough_plot_ts(trough_plot_index) = trough_ts(i-1);
    trough_plot_value(trough_plot_index) = trough_value(i-1);
    trough_plot_flag = flag_trough(i-1);
    trough_plot_index = trough_plot_index + 1;
end
if(count_peak >= floor((peak_index-2)/2) && count_trough >= floor((trough_index-2)/2))
    fprintf('%d,%d,%d,%d door closed\n',count_peak,floor((peak_index-2)/2),count_trough,floor((trough_index-2)/2));
end
legend('pitch','filtaz','corgx', 'gyrodyn', 'accdyn');
grid on
peak_value;
trough_value;
fprintf('--------------------------------------------------\n');

