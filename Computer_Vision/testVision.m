clear all;
close all;

% Connect to the ROS environment
ipaddress = '127.0.0.1';
robotType = 'Gazebo';
rosshutdown;
rosinit(ipaddress);

node = ros.Node('object_detect');

imSub = rossubscriber('/camera/color/image_raw');

figure;
sgtitle("Computer Vision Debugging")

while 1
    im = readImage(imSub.LatestMessage);

    block = getBlockMask(im); % Extract block/s
    stats = regionprops(block, 'BoundingBox', 'Centroid', 'FilledArea', 'PixelIdxList');
    
    subplot(3,2,[1 2]);
    imshow(block);
    
    subplot(3,2,[3 4 5 6]);
    imshow(im);

    for k=1:length(stats)
        bbox = stats(k).BoundingBox;
        width = bbox(3);
        height = bbox(4);
        filledArea = stats(k).FilledArea;
        if (filledArea > 1500)
            [X, Y] = getCentreCoordinates(stats(k));
            bbox = stats(k).BoundingBox;
            
            % Get Color
            color = getColor(block);
            
            % Get Shape
            blockImage = imcrop(im, bbox);
            shape = getShape(blockImage);
            
            % Add Bounding Rectangle
            rectangle('Position', bbox, 'EdgeColor', color, 'LineWidth', 3)
            text(X-width/2, Y, shape, 'fontsize', 10, 'FontWeight', 'bold', 'Color', color);
        end
    end
    % Get Color
    
    % Get Shape
    
    % Visualise
    pause(0.1);
end
% for k=1:length(stats)
%     % Disregard erroneous regions by only taking regions large
%     % enough to be a block
%     if (stats(k).FilledArea > 1500)
%         % Get X and Y coordinates
%         [X, Y] = getCentreCoordinates(stats(k));
% 
%         % Get Z coordinate
%         Z = xyz(X, Y, 3);
%         %Z = min(xyz(X-50:X+50,Y-50:Y+50,3));
% 
% 
%         fprintf('X = %f, Y = %f, Z = %f\n', X, Y, Z);
% 
%         X = -X/640*(640*500/480)/1000 + 1/3;
%         Y = 0.25 - Y/480*(480*500/480)/1000 + 0.5;
%         Z = -(Z - 0.575) + 0.25;
% 
%         % Get shape
%         blockImage = imcrop(testIm, stats(k).BoundingBox);
%         [Shape] = getShape(blockImage);
% 
%         % Print out all obtained information
%         fprintf('Block num %d:\n', k);
%         fprintf('X = %d, Y = %d, Z = %d\n', X, Y, Z);
%         fprintf('Shape: %s\n', Shape);
%         %plot(X, Y, 'bx', 'MarkerSize', 8, 'LineWidth', 2);
% 
%         % Publish message on topic '/msg/computer_vision'
%         visionpub = rospublisher('/msg/computer_vision', 'std_msgs/String');
%         msg = rosmessage(visionpub);
%         msg.Data = sprintf("%s %f %f %f\n", Shape, X, Y, Z);
%         send(visionpub, msg);
%         fprintf('Sent: %s\n', msg.Data);
%     end
% end