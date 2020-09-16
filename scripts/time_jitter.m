clc
clear all
a = load('~/.ros/time_debug.txt');
spiral = load('~/.ros/spiral_debug.txt');
verify = load('~/.ros/verify_debug.txt');

figure
plot(a(:,2))
legend('model update time')

figure
plot(a(:,3))
hold on
plot(a(:,4))
plot(a(:,5))
plot(a(:,7))
plot(a(:,19))
legend('joint_trajectory_action_server_', 'assemble_approach_action_server_', 'assemble_spiral_action_server_', ...
 'assemble_verify_action_server_','assemble_probe_action_server_' )