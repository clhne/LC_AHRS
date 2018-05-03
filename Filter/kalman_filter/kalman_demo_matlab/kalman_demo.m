clc;
clear all;
close all;
% ��ʼ������
n_iter = 100;       %��������n_iter��ʱ��
sz = [n_iter, 1]; 
x = 24;             % �¶ȵ���ʵֵ
Q = 4e-4;           % ���¶�Ԥ��ֵ�ķ���
R = 0.25;           % ���������Ӧ�¶ȼƵĲ�������
T_start = 23.5;     %�¶ȳ�ʼ����ֵ
delta_start = 1;    %�¶ȳ�ʼ���Ʒ���
z = x + sqrt(R)*randn(sz); 
% z���¶ȼƵĲ������������ʵֵ�Ļ����ϼ����˷���Ϊ0.25�ĸ�˹������
% ��ʼ������
state_kalman=zeros(sz); 
% ���¶ȵĹ���ֵ������kʱ�̣�����¶ȼƵ�ǰ����ֵ��k-1ʱ��Ԥ��ֵ���õ������չ���ֵ
variance_kalman=zeros(sz);         % ����ֵ�ķ���
state_pre=zeros(sz); % ���¶ȵ�Ԥ��
variance_pre=zeros(sz);    % Ԥ��ֵ�ķ���
K=zeros(sz);         % ����������
state_kalman(1) = T_start;   %�¶ȹ���ֵ��ʼ��
variance_kalman(1) =delta_start;   %����ֵ�����ʼ��
%
%��ʼ��������
for k = 2:n_iter
state_pre(k) = state_kalman(k-1);
%����һʱ�̵����Ź���ֵ����Ϊ�Ե�ǰʱ�̵��¶ȵ�Ԥ��
variance_pre(k) = variance_kalman(k-1)+Q;
%Ԥ��ķ���Ϊ��һʱ���¶����Ź���ֵ�ķ������˹��������֮��
%
%���㿨��������
K(k) = variance_pre(k)/( variance_pre(k)+R ); 
%
%��ϵ�ǰʱ���¶ȼƵĲ���ֵ������һʱ�̵�Ԥ�����У�����õ�У��������Ź��ơ�������ֱ�Ӳ�������CΪ1.
state_kalman(k) = state_pre(k)+K(k)*(z(k)-state_pre(k)); 
variance_kalman(k) = (1-K(k))*variance_pre(k); 
%�������չ���ֵ�ķ���������һ�ε���
end
%��ͼ
FontSize=14;
LineWidth=3;
figure();
plot(z,'k+'); %�����¶ȼƵĲ���ֵ
hold on;
plot(state_kalman,'b-','LineWidth',LineWidth) %�������Ź���ֵ
hold on;
plot(x*ones(sz),'g-','LineWidth',LineWidth); %������ʵֵ
legend('�¶Ȳ���ֵ', 'Kalman����ֵ', '��ʵֵ');
xl=xlabel('ʱ��(����)');
yl=ylabel('�¶�');
set(xl,'fontsize',FontSize);
set(yl,'fontsize',FontSize);
hold off;
set(gca,'FontSize',FontSize);
figure();
valid_iter = [2:n_iter]; % variance_pre not valid at step 1
plot(valid_iter,variance_kalman([valid_iter]),'LineWidth',LineWidth); %�������Ź���ֵ�ķ���
legend('Kalman���Ƶ�������');
xl=xlabel('ʱ��(����)');
yl=ylabel('��^2');
set(xl,'fontsize',FontSize);
set(yl,'fontsize',FontSize);
set(gca,'FontSize',FontSize);
