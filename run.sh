xhost +local:root

docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all,graphics \
    -e DISPLAY=$DISPLAY \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -v ./ros_ws/:/root/ros_workspace \
    --device=/dev/input \
    --name lab1 \
    ros:livelab1 bash