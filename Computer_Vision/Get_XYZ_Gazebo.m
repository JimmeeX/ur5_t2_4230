% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% This is a script to run all image processing steps and integrate into the
% other processes in this project
% The image processing unit returns a string containing [shape X-pixel
% Y-pixel Z-distance_from_camera]
% Written by Rowena Dai | z5075936

clear all;
close all;

% Connect to the ROS environment
ipaddress = '192.168.1.110';
robotType = 'Gazebo'
rosshutdown;
rosinit(ipaddress);

% Subscribe to the necessary topics (RGB Image, Image Depth, Break Beam
% Sensor)
imSub = rossubscriber('/camera/color/image_raw');
pcSub = rossubscriber('/camera/depth/points');
beamSub = rossubscriber('/break_beam_in_sensor', 'std_msgs/Bool');

% Continuous loop to run image processing
while 1
    
    % Get the status of the break beam
    % Find if the break beam has detected a block
    % Flag it if the break beam has been triggered
    data = receive(beamSub,5);
    detectblock = 0;
    if data.Data == 1 && breakbeamStatus == 0
        detectblock = 1;
    end
    breakbeamStatus = data.Data;
    
    % If there is a block in position, run image processing
    if (detectblock == 1)
        % Get the RGB image from the ROS environment
        testIm  = readImage(receive(imSub,5));
        
        % DEBUGGING - show the image obtained
        %figure(1)
        %imshow(testIm);
        %hold on;
        %title('RGB Image received from ROS');
        
        % Get the depth data from the ROS environment and convert data to
        % be in the same format as the RGB images
        ptcloud = receive(pcSub,5);
        ptcloud.PreserveStructureOnRead = true;
        xyz = readXYZ(ptcloud);

        % Isolate blocks in current image + clean up image
        block = getBlockMask(testIm); % Extract block/s
        %imshow(block)

        % Get properties of the individual blocks from the binary mask - currently
        %only needs 'BoundingBox' and 'Centroid'
        stats = regionprops(block, 'BoundingBox', 'Centroid', 'FilledArea', 'PixelIdxList');

        % For each region (there should only be 1 at a time), get the centre coordinates and shape
        for k=1:length(stats)
            % Disregard erroneous regions by only taking regions large
            % enough to be a block
            if (stats(k).FilledArea > 1500)
                % Get X and Y coordinates
                [X, Y] = getCentreCoordinates(stats(k));
                
                % Get Z coordinate
                Z = round(xyz(Y, X, 3),3);
                
                % Get shape
                blockImage = imcrop(testIm, stats(k).BoundingBox);
                [Shape] = getShape(blockImage);
                
                % Print out all obtained information
                fprintf('Block num %d:\n', k);
                fprintf('X = %d, Y = %d, Z = %d\n', X, Y, Z);
                fprintf('Shape: %s\n', Shape);
                %plot(X, Y, 'bx', 'MarkerSize', 8, 'LineWidth', 2);
                
                % Publish message on topic '/msg/computer_vision'
                visionpub = rospublisher('/msg/computer_vision', 'std_msgs/String');
                msg = rosmessage(visionpub);
                msg.Data = sprintf("%s %f %f %f\n", Shape, X, Y, Z);
                send(visionpub, msg);
                fprintf('Sent: %s\n', msg.Data);
            end
        end
    end
  

    

end

%figure(2)
%scatter3(ptcloud)
%title('Depth point cloud received from ROS');
%rostopic list;




