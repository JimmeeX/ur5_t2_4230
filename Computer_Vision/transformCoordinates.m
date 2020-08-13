function [Xt, Yt, Zt] = transformCoordinates(X,Y,Z)
im_width = 640;
im_height = 480;
conveyor_width = 0.5;
Y_dist_from_robot = 0.5;
height_of_camera = 0.575;
height_of_conveyor = 0.25;

dist_per_pixel = conveyor_width/im_height;

width_dist = im_width*dist_per_pixel;
height_dist = im_height*dist_per_pixel;

Xt = -X/im_width*width_dist + width_dist/2;
Yt = height_dist/2 - Y/im_height*height_dist + Y_dist_from_robot;
Zt = -(Z - height_of_camera) + height_of_conveyor;

end

