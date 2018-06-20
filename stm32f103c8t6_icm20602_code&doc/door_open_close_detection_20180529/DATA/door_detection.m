clc;clear all;close all;
fprintf('--------------------------------------------------\nMonotonicity detection:\n');
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
%dump_file='open_close_door_success5.txt';      % 波谷不单调             时间戳相差3个gryo周期
%dump_file='open_close_door_success6.txt';
%dump_file='open_close_door_success7_quiet.txt';
%dump_file='SaveWindows2018-5-29_10-48-17.txt';      % 波峰、波谷不单调，考虑关门时，门自身问题
%dump_file='SaveWindows2018-5-29_10-51-08.txt';      % 波谷不单调         
dump_file='SaveWindows2018-5-29_10-51-50.txt';      % 波峰、波谷不单调
%dump_file='SaveWindows2018-5-29_10-53-01.txt';      % 波峰、波谷不单调   时间戳相差2个gryo周期
%dump_file='SaveWindows2018-5-29_10-55-40.txt';      % 波峰、波谷不单调
%dump_file='SaveWindows2018-5-29_10-58-47.txt';       % 波峰、波谷不单调
%dump_file='SaveWindows2018_6_1_13-47-43.txt'; 
%dump_file='SaveWindows2018_6_1_13-56-26.txt';
%dump_file = 'SaveWindows2018_6_6_8-54-04.txt';
%dump_file = 'SaveWindows2018_6_6_9-20-36.txt';
data = load(dump_file);
ts = data(:,1); is_gyro_dyn = data(:,2); is_gyro_calib = data(:,3); is_acc_dyn = data(:,4);
is_acc_calib = data(:,5); roll = data(:,6); pitch = data(:,7); yaw = data(:,8); 
raw_ax = data(:,9);raw_ay = data(:,10);raw_az = data(:,11);filt_ax = data(:,12);
filt_ay = data(:,13);filt_az = data(:,14);raw_gx = data(:,15);raw_gy = data(:,16);
raw_gz = data(:,17);cor_gx = data(:,18);cor_gy = data(:,19);cor_gz = data(:,20); 
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
for i = 200:length(pitch)
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
                        crests_troughs_flag(crests_troughs_index) = 1; % 1代表波峰
                        crests_troughs_index = crests_troughs_index + 1;
                      elseif cur_k == 1 && pre_k == -1 && cor_gx(i-1) <= -0.125
                        crests_troughs_ts(crests_troughs_index) = ts(i-1);
                        crests_troughs_value(crests_troughs_index) = cor_gx(i-1);
                        if crests_troughs_value(crests_troughs_index) > -0.25
                            prev_dt = ts(i-1);
                        end
                        crests_troughs_flag(crests_troughs_index) = 0; % 0代表波谷
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