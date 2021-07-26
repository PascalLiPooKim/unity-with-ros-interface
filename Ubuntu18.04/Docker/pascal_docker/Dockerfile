FROM osrf/ros:melodic-desktop-full

#Add new sudo user - For display purposes
ENV USERNAME pascal
# Replace 1000 with your user/group id
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

USER $USERNAME

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo rm /bin/sh && sudo ln -s /bin/bash /bin/sh

RUN sudo apt-get update && sudo apt-get install -q -y python-catkin-tools

RUN sudo apt-get update && sudo apt-get install -q -y ros-melodic-hector-gazebo-plugins
 
RUN echo -----------------OpenCV from Source-------------
USER $USERNAME
RUN cd ~ && \
    sudo git clone https://github.com/opencv/opencv -b 3.4 && cd opencv && \
    sudo mkdir build && cd build && \
    sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D INSTALL_PYTHON_EXAMPLES=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D PYTHON_EXECUTABLE=$(which python2.7) \
-D BUILD_opencv_python2=ON \
-D CMAKE_INSTALL_PREFIX=/usr/local .. \
-D PYTHON2_EXECUTABLE=$(which python2.7) \
-D PYTHON2_INCLUDE_DIR=/usr/include/python2.7 \
-D WITH_GSTREAMER=ON \
-D BUILD_EXAMPLES=ON .. && \
    sudo make install

RUN echo -----------------Building RTABMAP---------------
COPY catkin_ws /home/$USERNAME/catkin_ws
COPY rtabmap /home/$USERNAME/rtabmap
RUN sudo apt-get update -y 
RUN sudo apt-get install -q -y ros-melodic-rtabmap ros-melodic-rtabmap-ros && \
    sudo apt-get remove -q -y ros-melodic-rtabmap ros-melodic-rtabmap-ros

RUN echo -----------------OTHER PKGS---------------------
USER $USERNAME
RUN sudo apt-get update -y && \
    sudo apt-get install python-pip -y && \
    sudo apt-get install ros-melodic-rosbridge-server && \
    pip install tornado -y && \
    pip install pymongo -y & \
    sudo apt-get install libopencv-dev -y && \
    sudo apt-get install ros-melodic-cv-bridge -y && \
    sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio -y && \
    sudo apt-get install libgstrtspserver-1.0-dev gstreamer1.0-rtsp -y && \
    sudo apt-get install ros-melodic-husky-simulator -y
RUN sudo apt-get install ros-melodic-rosbridge-server -y

RUN echo -------------------SOURCING---------------------
USER $USERNAME
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

RUN sudo usermod -a -G audio $USERNAME

COPY px4_entrypoint.sh /
ENTRYPOINT ["/px4_entrypoint.sh"]

CMD ["sh"]

