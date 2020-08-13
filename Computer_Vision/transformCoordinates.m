function [Xt, Yt] = transformCoordinates(X,Y)
im_width = 640;
im_height = 480;
conveyor_width = 0.5;
Y_dist_from_robot = 0.5;

dist_per_pixel = conveyor_width/im_height;

width_dist = im_width*dist_per_pixel;
height_dist = im_height*dist_per_pixel;

Xt = -X/im_width*width_dist + width_dist/2;
Yt = height_dist/2 - Y/im_height*height_dist + Y_dist_from_robot;
%         Z = -(Z - 0.575) + 0.25;

end

