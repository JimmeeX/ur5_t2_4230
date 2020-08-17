clear all;
close all;

% Connect to the ROS environment
ipaddress = '192.168.0.6'; % Must update
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
            color = getColor(im, X, Y);           

            % Get Shape
            blockImage = imcrop(im, bbox);
%             shape = getShape(blockImage);
            [shape,circ] = getIconShape(blockImage);
            
            % Transform coordinates
            [Xt, Yt] = transformCoordinates(X, Y);
            
            % Add Bounding Rectangle
            rectangle('Position', bbox, 'EdgeColor', 'b', 'LineWidth', 3)
            hold on
            plot(X, Y, 'bx', 'MarkerSize', 8)
            hold off
            
%             text(X-width/2, Y-height, strcat(color, {' '}, shape), ...
%                 'fontsize', 10, 'FontWeight', 'bold', 'Color', 'm');
            
            text(X-width/2, Y-height, strcat(color, {' '}, shape, {' | '}, num2str(circ)), ...
                'fontsize', 10, 'FontWeight', 'bold', 'Color', 'm');
            
            text(X-width/2, Y+height, ...
                strcat(num2str(Xt), {', '}, num2str(Yt)), ...
                'fontsize', 10, 'FontWeight', 'bold', 'Color', 'y');
            
        end
    end

    pause(0.1);
end