quaternion=importdata('quaternion.txt');
time = quaternion(:,5);
figure('Name', 'Quaternion Data');

hold on;
plot(time, quaternion(:,1), 'r');
plot(time, quaternion(:,2), 'g');
plot(time, quaternion(:,3), 'b');
plot(time, quaternion(:,4), 'black');
legend('0','X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Quaternion');
title('Quaternion');
hold off;
%linkaxes(axis, 'x');