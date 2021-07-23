XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
USER=pascal
touch $XAUTH
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --user=$pascal \
	--name pascal_container \
	--runtime nvidia \
	--volume "$(pwd)"/catkin_ws:/home/$USER/catkin_ws/ \
	--volume "$(pwd)"/rtabmap:/home/$USER/rtabmap \
	--volume "$(pwd)"/gst-rtsp-server:/home/$USER/gst-rtsp-server \
	--privileged \
	--net=host \
	--device /dev/snd \
	 eeeb843780f3 "$@"
