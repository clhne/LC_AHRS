clc;
clear all
close all
data1 = load('accfilterSaveWindows2018_4_27_13-44-20.TXT');
t = 1:length(data1);
plot(t,data1);
