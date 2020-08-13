clear all;
close all;

ipaddress = '192.168.1.18';
robotType = 'Gazebo';
rosshutdown;
rosinit(ipaddress);
blockposes = rossubscriber('/gazebo/link_states');
pause(2);
posdata = receive(blockposes,10);
imSub = rossubscriber('/camera/color/image_raw');
pcSub = rossubscriber('/camera/depth/points');

testIm  = readImage(imSub.LatestMessage);
imshow(testIm);
rostopic list;