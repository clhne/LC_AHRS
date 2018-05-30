clc;
clear alll;
close all;
% x = randn(3,1);
% x = randperm(10,3);
x = 2:4;
fun=@(x)[x(2);x(3);x(1)*(x(2)+x(3))];
[z, A] = jaccsd(fun,x)