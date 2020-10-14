docker run -it \
    --name="ur5_t2_4230_app_test" \
    --env="DISPLAY=$IP:0" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ur5_app:latest