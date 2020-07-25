function [X, Y] = getCentreCoordinates(region)
%Initialise variables
X = 0;
Y = 0;

%Check that the specified area is large enough to be a block
if region.FilledArea > 100
    %Get the centroid of the block (which should be the centre of the
    %block)
    centres = region.Centroid;
end

%Return the X & Y coordiates relative to the corner of the image
X = round(centres(1));
Y = round(centres(2));

end

