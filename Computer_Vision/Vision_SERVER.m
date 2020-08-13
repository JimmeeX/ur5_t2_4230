% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% This is a script to run all image processing steps and integrate into the
% other processes in this project
% The image processing unit returns a string containing [shape X-pixel
% Y-pixel Z-distance_from_camera]
% Written by Rowena Dai | z5075936

clear all;
close all;

% Connect to the ROS environment
ipaddress = '127.0.0.1';
robotType = 'Gazebo';
rosshutdown;
rosinit(ipaddress);

% Subscribe to the necessary topics (RGB Image, Image Depth, Break Beam
% Sensor)

beamSub = rossubscriber('/break_beam_in_sensor', 'std_msgs/Bool');

visionserver = rossvcserver('/vision/detect_object', 'rosbridge_library/SendBytes', @getData);
testclient = rossvcclient('/vision');

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
        testreq = call(testclient);
        disp(testreq.Str)
    end

end


function response = getData(~, ~, response)
    % Image Processing Variables
    BLOCK_AREA_THRESHOLD = 1500;

    % Transform Variables
    IM_WIDTH = 640;
    IM_HEIGHT = 480;
    CONVEYOR_HEIGHT = 0.5; % m
    PIXEL_TO_REAL = CONVEYOR_HEIGHT / IM_HEIGHT; % m/pixel


    imSub = rossubscriber('/camera/color/image_raw');
    pcSub = rossubscriber('/camera/depth/points');
        
    % Get the RGB image from the ROS environment
    im  = readImage(receive(imSub,5));

    % Get the depth data from the ROS environment and convert data to
    % be in the same format as the RGB images
    ptcloud = receive(pcSub,5);
    ptcloud.PreserveStructureOnRead = true;
    xyz = readXYZ(ptcloud);

    % Isolate blocks in current image + clean up image
    block = getBlockMask(im); % Extract block/s

    % Get properties of the individual blocks from the binary mask - currently
    %only needs 'BoundingBox' and 'Centroid'
    stats = regionprops(block, 'BoundingBox', 'Centroid', 'FilledArea', 'PixelIdxList');
    
    % For each region (there should only be 1 at a time), get the centre coordinates and shape
    for k=1:length(stats)
        bbox = stats(k).BoundingBox;

        % Disregard erroneous regions by only taking regions large
        % enough to be a block
        
        if (stats(k).FilledArea > BLOCK_AREA_THRESHOLD)
            % Get X and Y coordinates
            [X, Y] = getCentreCoordinates(stats(k));

            % Get Color
            color = getColor(block);
            
            % Get Shape
            blockImage = imcrop(im, bbox);
            shape = getShape(blockImage);
            
            % Get Z coordinate
            Z = xyz(X, Y, 3);
            
            % Transform coordinates into robot's frame of reference
            %fprintf('X = %f, Y = %f, Z = %f\n', X, Y, Z);
            X = -X/640*(640*500/480)/1000 + 1/3;
            Y = 0.25 - Y/480*(480*500/480)/1000 + 0.5;
            Z = -(Z - 0.575) + 0.25;

            % Print out all obtained information
            fprintf('Block num %d:\n', k);
            fprintf('X = %d, Y = %d, Z = %d\n', X, Y, Z);
            fprintf('Shape: %s\n', shape);
            %plot(X, Y, 'bx', 'MarkerSize', 8, 'LineWidth', 2);

            % Return server message
            response.Data = sprintf("%s %s %f %f %f\n", shape, color, X, Y, Z);
            fprintf('Sent: %s\n', response.Data);
        end
    end

end

%figure(2)
%scatter3(ptcloud)
%title('Depth point cloud received from ROS');
%rostopic list;






