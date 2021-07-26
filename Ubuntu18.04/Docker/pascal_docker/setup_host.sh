XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
USER=myNewUserName
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
	--name="px4_sim_container" \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --user=$myNewUserName \
	--name husky_container \
	--runtime nvidia \
	--volume "$(pwd)"/catkin_ws:/home/$USER/catkin_ws/ \
	--volume "$(pwd)"/rtabmap:/home/$USER/rtabmap \
	--network host \
	 3a88c0b7f440 "$@"
