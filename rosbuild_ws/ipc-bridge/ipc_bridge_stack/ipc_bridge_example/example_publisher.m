% IPC-Bridge Example Publisher
% 1/24/2011
% Ben Cohen
clear all;
clc;

% add the ipc_bridge_matlab binaries to your path
%[a, p] = system('rospack find ipc_geometry_msgs');
addpath(strcat('/home/schmsimo/Projects/ps3eye/rosbuild_ws/ipc-bridge/ipc_msgs/ipc_geometry_msgs', '/bin')); 


% create a publisher that publishes a geometry_msgs/Twist message
pid=geometry_msgs_Twist('connect','publisher','example_module','twist');

% create an empty geometry_msgs/Twist message structure
msg=geometry_msgs_Twist('empty');

% assign some values
msg.linear.x = 1.0;
msg.linear.y = 2.0;
msg.linear.z = 3.0;

% publish forever
while (1)
    geometry_msgs_Twist('send',pid,msg);
    msg.linear.x = msg.linear.x + 1;
    msg.linear.y = msg.linear.y + 1;
    msg.linear.z = msg.linear.z + 1;
end
