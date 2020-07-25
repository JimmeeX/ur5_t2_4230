clear;clc;close all;

% Specify folder name with images
folderName = '_Multiple Blocks\';
folder = dir(folderName);
numFiles = length(folder); % Get number of parameters in folder struct

% First 2 parameters don't count i.e. 
% folder(1).name = '.', folder(2).name = '..'
for i = 3:7 %numFiles for all images in folder
    % Get current image file name
    fileName = folder(i).name;
    % Get file's location relative to current directory
    file = strcat(folderName,fileName);
    % Read the image file
    image= imread(file);
    
    % Isolate blocks in current image + clean up image
    block = getBlockMask(image); % Extract block/s
    block = imerode(block, strel('sphere',5));
    block = imdilate(block, strel('sphere',5));
    
    %Create a copy of the image
    filt_image = image;
    
    % Display image outputs
    figure(i-2);
    imshow(filt_image); title('Block/s');
    sgtitle(['Image: ', fileName]);
    hold on;
    
    %Get properties of the individual blocks from the binary mask - currently
    %only needs 'Centroid'
    stats = regionprops(block, 'BoundingBox', 'Centroid', 'FilledArea', 'PixelIdxList');
    
    %For each region, get the centre coordinates
    %Plot the coordinates on the image
    for k=1:length(stats)
        [X, Y] = getCentreCoordinates(stats(k));
        plot(X, Y, 'rx', 'MarkerSize', 8, 'LineWidth', 2);
        fprintf('Figure %d, Block num %d:\n', i-2, k);
        fprintf('X = %d\n', X);
        fprintf('Y = %d\n', Y);
    end
    
end