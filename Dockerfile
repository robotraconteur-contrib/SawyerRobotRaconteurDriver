FROM ubuntu:focal
WORKDIR /app

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install wget sudo software-properties-common curl cmake g++ build-essential ninja-build \
    autoconf automake libtool bison libpcre3-dev ca-certificates -y
RUN sudo add-apt-repository ppa:robotraconteur/ppa -y \
    && sudo apt-get update \
    && sudo apt-get install librobotraconteur-net-native -y

RUN wget https://dot.net/v1/dotnet-install.sh -O dotnet-install.sh \
    && chmod +x dotnet-install.sh \
    && ./dotnet-install.sh --channel 6.0 --install-dir /opt/dotnet

ENV PATH="${PATH}:/opt/dotnet"
ENV DOTNET_ROOT=/opt/dotnet

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && sudo apt update \
    && sudo apt install ros-noetic-ros-base -y \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN mkdir -p swig_build_dir \
    && cd swig_build_dir \
    && curl -s -L https://github.com/swig/swig/archive/refs/tags/v4.0.2.tar.gz --output swig-src.tar.gz \
    && tar xf swig-src.tar.gz \
    && cd swig-4.0.2 \
    && ./autogen.sh \
    && ./configure \
    && make \
    && sudo make install


ENV PATH="${PATH}:/opt/ros/noetic/bin"
ENV ROS_DISTRO=noetic
ENV ROS_PYTHON_VERSION=3
ENV ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ENV ROS_ROOT=/opt/ros/noetic/share/ros
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/ros/noetic/lib/x86_64-linux-gnu:/opt/ros/noetic/lib:/opt/ros/noetic/lib/python3/dist-packages:/usr/local/lib"
ENV CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/opt/ros/noetic"

COPY ./ros_csharp_interop /app/ros_csharp_interop

RUN cd ros_csharp_interop \
    && cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --config Release \
    && cmake --build build --target ros_csharp_interop \
    && sudo cmake --build build --target install

COPY ./SawyerRobotRaconteurDriver /app/SawyerRobotRaconteurDriver
RUN cd SawyerRobotRaconteurDriver \
    && dotnet publish --framework=net6.0 --self-contained false -o /opt/sawyer_robotraconteur_driver/bin

COPY ./SawyerRobotRaconteurDriver/config/*.yml /config/

RUN rm -rf /app && apt-get clean && rm -rf /var/lib/apt/lists/*


ENV ROBOT_INFO_FILE=/config/sawyer_robot_default_config.yml
ENV ROS_MASTER_URI=http://127.0.0.1:11311
ENV ROS_IP=127.0.0.1

CMD exec /opt/sawyer_robotraconteur_driver/bin/SawyerRobotRaconteurDriver --robot-info-file=$ROBOT_INFO_FILE
