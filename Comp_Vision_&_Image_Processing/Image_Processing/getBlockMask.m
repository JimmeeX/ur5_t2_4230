  
function [block] = getBlockMask(image)

% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% isolateIcon function isolates the block/s present in image file 'image'
% Written by Jason Jia Sheng Quek | z5117285
% Modified by Rowena Dai | z5075936 - returns binary mask only

% Convert RGB image to HSV colour space
image_HSV = rgb2hsv(image);

% Define thresholds for hue channel 1
hueMin = 0.000;
hueMax = 1.000;

% Define thresholds for saturation channel 2
satMin = 0.000;
satMax = 1.000;

% Define thresholds for value channel 3
valMin = 0.245;
valMax = 0.556;

% Use the thresholds to simultaneously apply multiple binary masks to image
block_mask = (image_HSV(:,:,1) >= hueMin ) & (image_HSV(:,:,1) <= hueMax) & ...
          (image_HSV(:,:,2) >= satMin ) & (image_HSV(:,:,2) <= satMax) & ...
          (image_HSV(:,:,3) >= valMin ) & (image_HSV(:,:,3) <= valMax);

% Return the binary mask
block = block_mask; % Make a copy of 'image_HSV'

end