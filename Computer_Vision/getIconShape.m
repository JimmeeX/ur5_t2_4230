function [shape,circ] = getIconShape(blockImage)

% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% Given a cropped image of a single block surface, getIconShape function
% returns the shape of the icon (circle/square/triangle)
% Written by Jason Jia Sheng Quek | z5117285

    % Convert cropped RGB image to HSV colour space
    image_HSV = rgb2hsv(blockImage);

    % Define thresholds for hue channel 1
    hueMin = 0.000;
    hueMax = 1.000;

    % Define thresholds for saturation channel 2
    satMin = 0.400;
    satMax = 1.000;

    % Define thresholds for value channel 3
    valMin = 0.000;
    valMax = 1.000;

    % HSV colour thresholding produces a binary image 'icon_mask'
    icon_mask = (image_HSV(:,:,1) >= hueMin ) & (image_HSV(:,:,1) <= hueMax) & ...
              (image_HSV(:,:,2) >= satMin ) & (image_HSV(:,:,2) <= satMax) & ...
              (image_HSV(:,:,3) >= valMin ) & (image_HSV(:,:,3) <= valMax);
    
    % Area opening and erosion removes artefacts like icons that are not
    % directly facing the camera
    icon_mask = bwareaopen(icon_mask, 150);   
    SE = strel('square',3); % width in pixels
    icon_mask = imerode(icon_mask, SE);

    % Extract detected icon's circularity
    s = regionprops(icon_mask,'Circularity');
    circ = s.Circularity;
    
    % Identify and return icon shape based on 'circ' range
    if circ >= 0.95 && circ <= 1.10
        shape = 'circle';
    elseif circ >= 0.86 && circ <= 0.94
        shape = 'square';
    elseif circ >= 0.64 && circ <= 0.78
        shape = 'triangle';
    else
        shape = '';
    end

end