xhost +
docker run -it -d --gpus all --name mrslam --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /tmp/catkin_ws:/root/workspace/src --env="QT_X11_NO_MITSHM=1" mrslam:noetic
