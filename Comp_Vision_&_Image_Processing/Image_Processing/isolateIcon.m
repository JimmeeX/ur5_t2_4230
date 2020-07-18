function [icon] = isolateIcon(image)

% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% isolateIcon function isolates the icon/s present in image file 'image'
% Written by Jason Jia Sheng Quek | z5117285

% Convert RGB image to HSV colour space
image_HSV = rgb2hsv(image);

% Define thresholds for hue channel 1
hueMin = 0.000;
hueMax = 1.000;

% Define thresholds for saturation channel 2
satMin = 0.400;
satMax = 1.000;

% Define thresholds for value channel 3
valMin = 0.000;
valMax = 1.000;

% Use the thresholds to simultaneously apply multiple binary masks to image
icon_mask = (image_HSV(:,:,1) >= hueMin ) & (image_HSV(:,:,1) <= hueMax) & ...
          (image_HSV(:,:,2) >= satMin ) & (image_HSV(:,:,2) <= satMax) & ...
          (image_HSV(:,:,3) >= valMin ) & (image_HSV(:,:,3) <= valMax);

% Apply mask to all HSV channels of 'image_HSV'
hsv_BinMasks = cat(3, icon_mask, icon_mask, icon_mask);
icon = image; % Make a copy of 'image_HSV'

% Via logical indexing, use inverted hsv_BinMasks to replace 'IM_HSV' pixel
% elements of logical index 1 with the value 0 to isolate the block
icon(~hsv_BinMasks) = 0;

end