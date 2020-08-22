function [Xt, Yt, Zt] = transformCoordinates(X,Y,Z)
im_width = 640;
im_height = 480;
conveyor_width = 0.45; % 0.5
Y_dist_from_robot = 0.5;
% height_of_camera = 0.575;
height_of_conveyor = 0.25;
height_of_robot = 0.3;



spacing = 0.001;

dist_per_pixel = conveyor_width/im_height;

width_dist = im_width*dist_per_pixel;
height_dist = im_height*dist_per_pixel;

Yt = -X/im_width*width_dist + width_dist/2;
Xt = height_dist/2 - Y/im_height*height_dist + Y_dist_from_robot;
% Zt = -(Z - height_of_camera) + height_of_conveyor;

% Swap Xt and Yt, make them both negative, and set Zt = -0.05 (to cater to robot frame)
Xt = -Xt;
Yt = -Yt;
Zt = height_of_conveyor-height_of_robot + spacing; % Robot -> z=0.3; Top of Conveyor -> z=0.2; Top of object -> 0.2+0.05

end

