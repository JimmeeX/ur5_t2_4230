function shape = getShape(blockImage)
% MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
% getShape takes in a cropped image of the surface of the block and
% processes it to find the shape of the icon
% Written by Rowena Dai | z5075936
    
    [height, width, ~] = size(blockImage);
    grayimage = rgb2gray(blockImage);
    corners = detectFASTFeatures(grayimage, 'MinQuality', 0.75, 'ROI', [10, 10, width-20, height-20]);
    %imshow(grayimage); hold on;
    %plot(corners);
    
    if corners.Count == 4
        shape = 'square';
    elseif corners.Count == 3
        shape = 'triangle';
    else
        shape = 'circle';
    end
    

end

