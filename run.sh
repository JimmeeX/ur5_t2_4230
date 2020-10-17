xhost + # Disable access control so that docker containers can communicate with x-server on host (so GUIs will work)

docker-compose up --build # Run Docker Containers

xhost - # Enable access control for security

# DISPLAY=$DISPLAY allows container to connect to X server
# QT_X11_NO_MITSHM=1 stops Qt from using MIT-SHM X11 extension
# Volume is also needed to connect to X server
# GPUs all allows container to access host's GPU (unfortunately Gazebo requires GPU to launch properly)
# docker run -it \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --gpus="all" \
#     test-gazebo:latest \
#     roslaunch ur5_t2_4230 ur5_world.launch