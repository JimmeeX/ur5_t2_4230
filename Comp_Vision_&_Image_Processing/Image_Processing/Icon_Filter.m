% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% Test Program for isolateIcon and isolateBlock functions
% Jason Jia Sheng Quek | z5117285

clear;clc;close all;
% Specify folder name with images
folderName = 'Orange Circle\';
folder = dir(folderName);
numFiles = length(folder); % Get number of parameters in folder struct

% First 2 parameters don't count i.e. 
% folder(1).name = '.', folder(2).name = '..'
for i = 3: numFiles
    % Get current image file name
    fileName = folder(i).name;
    % Get file's location relative to current directory
    file = strcat(folderName,fileName);
    % Read the image file
    image= imread(file);
    
    % Isolate features of current image
    block = isolateBlock(image); % Extract block/s
    icon = isolateIcon(image); % Extract icon/s on block/s    
    
    % Display function outputs
    figure(i-2);
    subplot(1,2,1); imshow(block); title('Block/s');
    subplot(1,2,2); imshow(icon); title('Icon/s');
    sgtitle(['Image: ', fileName]);
end