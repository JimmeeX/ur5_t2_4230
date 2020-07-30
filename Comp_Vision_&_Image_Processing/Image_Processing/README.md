Image Processing contains the MATLAB code responsible for feature identification and isolation

READING IMAGE DATA FROM ROS
Use Get_XY_Gazebo.m to connect to the ROS environment
- Remember to check and change IP address as necessary

Currently only reads the image once.


There are 3 MATLAB (.m) files of interest:
1. isolateBlock.m contains the isolateBlock function, which creates and applies a binary mask isolating block/s in the image
2. isolateIcon.m contains the isolateIcon function, which creates and applies a binary mask isolating icon/s in the image
3. Icon_Filter.m is the main test program for the isolateBlock and isolateIcon functions

The folders in this directory contain images categorised by:
1. Icon details (colour + shape) e.g. "Blue Circle"
2. Number of blocks. The "_Multiple Blocks" folder is the only folder with multiple blocks

These folders are referenced by the Icon_Filter.m test program with using relative directory reference. To run the test program
using a particular folder, type the folder name in the 'folderName' variable in Icon_Filter.m and terminate with a backslash (\\)
e.g. 'Blue Circle\\'

Extension - getting the centre coordinates of a block in an image:

1. getBlockMask.m takes in a camera image and obtains the binary mask for isolating blocks in the image
2. getCentreCoordinates.m takes in a struct (with properties from the regionprops function) and obtains the X and Y pixel coordinates of the centre of the block
3. Get_XY.m is the main test program
