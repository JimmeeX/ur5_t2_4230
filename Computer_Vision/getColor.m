function color = getColor(im, x, y)
pixelValue = im(x,y,:);

colors = ["red", "green", "blue"];

[value, index] = max(pixelValue);

disp(pixelValue);
disp(value);
disp(index);
disp(colors(index));
color = colors(index);

end