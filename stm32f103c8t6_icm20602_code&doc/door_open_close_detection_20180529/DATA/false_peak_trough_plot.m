clc;clear all;close all;
fprintf('--------------------------------------------------\nMonotonicity detection:\n');
%dump_file='false_peak_trough_062902.txt';
%dump_file='SaveWindows2018-6-29_14-50-53.txt';
%dump_file='SaveWindows2018-6-29_14-53-25.txt';
%dump_file='SaveWindows2018-6-29_14-56-39.txt';
%dump_file='SaveWindows2018-6-29_14-53-25.txt';
%dump_file='SaveWindows2018-7-2_16-31-33.txt';
%dump_file='SaveWindows2018-7-2_16-33-27.txt';
%dump_file='SaveWindows2018-7-2_16-57-45.txt';
%dump_file='SaveWindows2018_7_2_19-28-02.txt';
%dump_file='SaveWindows2018_7_2_19-37-55.txt';
%dump_file='SaveWindows2018_7_3_10-35-26.txt';    % t=34841 false
%dump_file='SaveWindows2018_7_3_11-26-24.txt';    % t=16040
%dump_file='SaveWindows2018_7_3_11-41-46.txt';    % t=29760
%dump_file='SaveWindows2018_7_3_11-54-10.txt';
%dump_file='SaveWindows2018_7_3_13-49-11.txt';     %t=6607
%dump_file='SaveWindows2018_7_3_14-00-07.txt';
%dump_file='SaveWindows2018_7_4_9-20-17.txt';       %t=12934
%dump_file='SaveWindows2018_7_4_10-09-39.txt';
%dump_file='SaveWindows2018-7-4_11-12-05.txt';
%dump_file='SaveWindows2018-7-4_11-12-37.txt';      % t=47308 index should not +1
%dump_file='SaveWindows2018-7-4_11-13-28.txt';      % true except index num
%dump_file='SaveWindows2018-7-4_11-14-45.txt';
dump_file='SaveWindows2018-7-4_11-15-49.txt';

%dump_file='SaveWindows2018_7_4_15-49-20.txt';      % ture
data = load(dump_file);
ts = data(:,7); is_gyro_dyn = data(:,8); is_acc_dyn = data(:,9);
is_acc_calib = data(:,10); pitch = data(:,6);  
raw_gx = data(:,11);cor_gx = data(:,12);
plot(ts, cor_gx, 'b');
hold on;
plot(ts, pitch, 'r');
hold on;
plot(ts, is_gyro_dyn, 'k');
hold on;
plot(ts, is_acc_dyn, 'c');
N = length(ts);
crests_troughs_index = 1;
crests_troughs_init = 0;
prev_dt = 0;
cur_dt = 0;
count_monotonicity = 0;
for i = 700:length(pitch)
    if  abs(pitch(i)) >= 4
        else if abs(pitch(i)) < 4
                if cor_gx(i - 1) < cor_gx(i)
                   cur_k = 1;
                elseif cor_gx(i - 1) > cor_gx(i)
                   cur_k = -1;
                else
                    continue;
                end
                if crests_troughs_init == 1
                    if cur_k ~= pre_k
                      if cur_k == -1 && pre_k == 1 && cor_gx(i-1) >= 0.125
                        crests_troughs_ts(crests_troughs_index) = ts(i-1);
                        crests_troughs_value(crests_troughs_index) = cor_gx(i-1);
                        if crests_troughs_value(crests_troughs_index) < 0.25
                            prev_dt = ts(i-1);
                        end
                        crests_troughs_flag(crests_troughs_index) = 1; % 1��������
                        crests_troughs_index = crests_troughs_index + 1;
                      elseif cur_k == 1 && pre_k == -1 && cor_gx(i-1) <= -0.125
                        crests_troughs_ts(crests_troughs_index) = ts(i-1);
                        crests_troughs_value(crests_troughs_index) = cor_gx(i-1);
                        if crests_troughs_value(crests_troughs_index) > -0.25
                            prev_dt = ts(i-1);
                        end
                        crests_troughs_flag(crests_troughs_index) = 0; % 0��������
                        crests_troughs_index = crests_troughs_index + 1;
                      end
                    end
                else
                    crests_troughs_init = 1;
                end
                if is_gyro_dyn(i-1) == 0 && is_acc_dyn(i-1) == 0
                    cur_dt = ts(i-1)
                    break;
                end
                pre_k = cur_k;
            else
                crests_troughs_index = 1;
                crests_troughs_init = 0;
            end
     end
end


count_monotonicity = 0;
for j = 2:crests_troughs_index-1
       if abs(crests_troughs_value(j)) <= abs(crests_troughs_value(j-1))
%            fprintf('%f, %f\n',crests_troughs_value(i-1),crests_troughs_value(i));
           count_monotonicity = count_monotonicity + 1;
       else
           continue;
       end
end

if(count_monotonicity >= floor(crests_troughs_index * 0.5))
    if(cur_dt - prev_dt > 300)
        fprintf('Data is Monotonicity! %d,%d\n',count_monotonicity,floor(crests_troughs_index*0.5));
    end

else
    fprintf('Data is NOT Monotonicity! %d,%d\n',count_monotonicity,floor(crests_troughs_index*0.5));
end
crests_troughs_value
cur_dt
prev_dt
cur_dt - prev_dt
fprintf('--------------------------------------------------\n');
crests_troughs_filt_first = 0;
crests_troughs_filt_index = 1;
for i = 2:crests_troughs_index
    if crests_troughs_filt_first == 0
        if abs(crests_troughs_value(i-1)) > 0.1
            crests_troughs_filt_first = 1;
        else
            continue;
        end
    else
        if crests_troughs_filt_first == 1 && abs(crests_troughs_value(i-1)) < 0.1 && abs(crests_troughs_value(i)) < 0.1
            break;
        end
    end
    plot(crests_troughs_ts(i-1), crests_troughs_value(i-1), 'ko');
    crests_troughs_filt_ts(crests_troughs_filt_index) = crests_troughs_ts(i-1);
    crests_troughs_filt_value(crests_troughs_filt_index) = crests_troughs_value(i-1);
    crests_troughs_filt_flag(crests_troughs_filt_index) = crests_troughs_flag(i-1);
    crests_troughs_filt_index = crests_troughs_filt_index + 1;
end
legend('corgx','pitch', 'gyrodyn', 'accdyn', 'crest', 'trough');
grid on