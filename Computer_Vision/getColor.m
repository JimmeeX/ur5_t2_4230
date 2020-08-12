function color = getColor(im)
% pixelValue = im(x,y,:);

colors = ["red", "green", "blue"];

% [value, index] = max(pixelValue);
% 
% disp(pixelValue);
% disp(value);
% disp(index);
% disp(colors(index));
% color = colors(index);

color = colors(randi(3));

end