## Custom Dockerfile
FROM neuroidss/ubuntu-xfce-vnc:18.04
ENV REFRESHED_AT 2018-03-18

## Install a gedit
USER 0
RUN apt-get update \
    && apt-get -y install apt-utils \
    && apt-get -y install xz-utils lsb-release cmake g++ unzip sudo software-properties-common git \
    && apt-get -y install doxygen make cmake gcc g++ libexpat1-dev libncurses5-dev libboost-dev libboost-thread-dev libboost-regex-dev libboost-chrono-dev libboost-filesystem-dev ninja-build libzzip-dev libxerces-c-dev libgtest-dev  \
    && apt-get -y install libogre-1.9-dev libboost-chrono-dev wget doxygen make automake  autoconf cmake unzip gcc g++ libgtk2.0-dev libglade2-dev gfortran  libgsl0-dev  libreadline-dev  libzzip-dev  libtool  libxaw7-dev  libpcre3-dev  libfreeimage-dev  libglu1-mesa-dev  libalut-dev  libvorbis-dev  libncurses5-dev  libeigen3-dev libcegui-mk2-dev  libois-dev  libboost-dev  libboost-thread-dev  libboost-regex-dev  libboost-filesystem-dev  liblua5.1-0-dev  libitpp-dev  libsqlite0-dev  libfftw3-dev	 python-dev  python-numpy  sqlite  \
    && apt-get -y install freeglut3-dev \
    && apt-get -y install libqt4-designer libqt4-opengl libqt4-svg libqtgui4 libqtwebkit4 \
    && apt-get -y install libsuitesparse-dev libsuperlu-dev libptscotch-dev libmetis-dev \
    && apt-get -y install libqt4-dev \
    && apt-get -y install cmake gcc gfortran libhwloc-dev libscotch-dev libopenblas-dev liblapacke-dev python-numpy \
    && ulimit -s unlimited \
    && wget http://openvibe.inria.fr/pub/src/openvibe-2.2.0-src.tar.xz \
    && tar -xvf ./openvibe-2.2.0-src.tar.xz \
    && rm ./openvibe-2.2.0-src.tar.xz \
    && sudo apt-get install cmake libgtest-dev \
    && cd /usr/src/gtest \
    && sudo cmake CMakeLists.txt \
    && sudo make \
    && sudo cp *.a /usr/lib 

ADD ./openvibe-2.2.0-src/extras/CMakeLists.txt /headless/openvibe-2.2.0-src/extras/
ADD ./openvibe-2.2.0-src/sdk/CMakeLists.txt /headless/openvibe-2.2.0-src/sdk/
ADD ./openvibe-2.2.0-src/extras/applications/platform/tracker/src/ParallelExecutor.cpp /headless/openvibe-2.2.0-src/extras/applications/platform/tracker/src/

RUN cd openvibe-2.2.0-src  \
    && ./build.sh \
    && apt clean all

#TODO submit patch to change sources
RUN wget https://github.com/neuroidss/FreeEEG32-alpha1.5/raw/master/OpenVIBE/openvibe-2.2.0-src_FreeEEG32-alpha1.5.tar.gz -O openvibe-2.2.0-src_FreeEEG32-alpha1.5.tar.gz \
    && tar -xzvf ./openvibe-2.2.0-src_FreeEEG32-alpha1.5.tar.gz \
    && cp -r ./openvibe-2.2.0-src_FreeEEG32-alpha1.5/extras/contrib/common/* openvibe-2.2.0-src/extras/contrib/common/ \
    && cp -r ./openvibe-2.2.0-src_FreeEEG32-alpha1.5/extras/contrib/plugins/server-drivers/* openvibe-2.2.0-src/extras/contrib/plugins/server-drivers/ \
    && cp -r ./openvibe-2.2.0-src_FreeEEG32-alpha1.5/extras/applications/platform/acquisition-server/share/* openvibe-2.2.0-src/extras/applications/platform/acquisition-server/share/ \
    && cp -r ./openvibe-2.2.0-src_FreeEEG32-alpha1.5/extras/applications/platform/acquisition-server/src/* openvibe-2.2.0-src/extras/applications/platform/acquisition-server/src/ \
    && rm ./openvibe-2.2.0-src_FreeEEG32-alpha1.5.tar.gz \
    && cd openvibe-2.2.0-src  \
    && ./build.sh


## switch back to default user
USER 1000
