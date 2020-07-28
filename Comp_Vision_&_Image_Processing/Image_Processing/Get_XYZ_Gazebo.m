clear all;
close all;

ipaddress = '192.168.1.110';
robotType = 'Gazebo'
rosshutdown;
rosinit(ipaddress);

blockposes = rossubscriber('/gazebo/link_states');
pause(2);
posdata = receive(blockposes,10);
imSub = rossubscriber('/camera/color/image_raw');
pcSub = rossubscriber('/camera/depth/points');

figure(1)
testIm  = readImage(receive(imSub,5));
imshow(testIm);
hold on;
title('RGB Image received from ROS');

ptcloud = receive(pcSub,5);
ptcloud.PreserveStructureOnRead = true;
xyz = readXYZ(ptcloud);

% Isolate blocks in current image + clean up image
block = getBlockMask(testIm); % Extract block/s

%imshow(block)

%Get properties of the individual blocks from the binary mask - currently
%only needs 'Centroid'
stats = regionprops(block, 'BoundingBox', 'Centroid', 'FilledArea', 'PixelIdxList');

%For each region, get the centre coordinates
%Plot the coordinates on the image
for k=1:length(stats)
    [X, Y] = getCentreCoordinates(stats(k));
    Z = round(xyz(Y, X, 3),3);
    plot(X, Y, 'bx', 'MarkerSize', 8, 'LineWidth', 2);
    fprintf('Block num %d:\n', k);
    fprintf('X = %d\n', X);
    fprintf('Y = %d\n', Y);
    fprintf('Z = %f\n', Z);
end

figure(2)
scatter3(ptcloud)
title('Depth point cloud received from ROS');
%rostopic list;




