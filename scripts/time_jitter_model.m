clc
clear all
close all
a = load('~/.ros/time_debug_model.txt');
for i = 1:9
    figure
    title(i)
    plot(a(:,i))
end