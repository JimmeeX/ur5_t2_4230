clear all;
close all;

ipaddress = '192.168.1.109';
robotType = 'Gazebo';
rosshutdown;
rosinit(ipaddress);

pause on
while (1)
    chatter = rossubscriber('/chatter/ToMatlab');
    data = receive(chatter,10);
    fprintf('Received: %s\n', data.Data);
    
    chatpub = rospublisher('/chatter/ToRos', 'std_msgs/String');
    msg = rosmessage(chatpub);
    msg.Data = strcat('Hello, this is MATLAB ', datestr(now, ' HH:MM:SS'));
    send(chatpub, msg);
    fprintf('Sent: %s\n', msg.Data);
    
    pause(3)
end
