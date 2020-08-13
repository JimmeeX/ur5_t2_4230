function color = getColor(image, X, Y)
    rgb = image(Y, X, :);
    
    [~, I] = max(rgb);
    
    switch I
        case 1
            color = 'red';
        case 2
            color = 'green';
        case 3
            color = 'blue';
    end
end

