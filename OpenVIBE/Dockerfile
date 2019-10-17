## Custom Dockerfile
FROM consol/ubuntu-xfce-vnc
ENV REFRESHED_AT 2018-03-18

## Install a gedit
USER 0
RUN apt-get update \
    && apt-get -y install xz-utils lsb-release cmake g++ unzip sudo software-properties-common git \
    && apt-get -y install doxygen make cmake gcc g++ libexpat1-dev libncurses5-dev libboost-dev libboost-thread-dev libboost-regex-dev libboost-chrono-dev libboost-filesystem1.58-dev ninja-build libzzip-dev libxerces-c-dev libgtest-dev \
    && apt-get -y install libgtk2.0-dev libglade2-dev \
    && apt-get -y install libogre-1.9-dev libboost-chrono1.58-dev wget doxygen make automake  autoconf cmake unzip gcc g++ libgtk2.0-dev libglade2-dev gfortran  libgsl0-dev  libreadline-dev  libzzip-dev  libtool  libxaw7-dev  libpcre3-dev  libfreeimage-dev  libglu1-mesa-dev  libalut-dev  libvorbis-dev  libncurses5-dev  libeigen3-dev libcegui-mk2-dev  libois-dev  libboost-dev  libboost-thread-dev  libboost-regex-dev  libboost-filesystem-dev  liblua5.1-0-dev  libitpp-dev  libsqlite0-dev  libfftw3-dev	 python-dev  python-numpy  sqlite \
    && wget http://openvibe.inria.fr/pub/src/openvibe-2.1.0-src.tar.xz \
    && tar -xvf ./openvibe-2.1.0-src.tar.xz \
    && rm ./openvibe-2.1.0-src.tar.xz \
    && cd openvibe-2.1.0-src \
    && ./install_dependencies.sh \
    && ./build.sh \
    && apt clean all

#TODO submit patch to change sources
RUN wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=1CouIzOqKzLHiPNkZ79GCLhN2CjgHR388'  -O openvibe-2.1.0-src_FreeEEG32-alpha.tar.gz \
    && tar -xzvf ./openvibe-2.1.0-src_FreeEEG32-alpha.tar.gz \
    && cp -r ./openvibe-2.1.0-src_FreeEEG32-alpha/extras/contrib/common/* openvibe-2.1.0-src/extras/contrib/common/ \
    && cp -r ./openvibe-2.1.0-src_FreeEEG32-alpha/extras/contrib/plugins/server-drivers/* openvibe-2.1.0-src/extras/contrib/plugins/server-drivers/ \
    && cp -r ./openvibe-2.1.0-src_FreeEEG32-alpha/extras/applications/platform/acquisition-server/share/* openvibe-2.1.0-src/extras/applications/platform/acquisition-server/share/ \
    && cp -r ./openvibe-2.1.0-src_FreeEEG32-alpha/extras/applications/platform/acquisition-server/src/* openvibe-2.1.0-src/extras/applications/platform/acquisition-server/src/ \
    && rm ./openvibe-2.1.0-src_FreeEEG32-alpha.tar.gz \
    && cd openvibe-2.1.0-src \
    && ./build.sh 


## switch back to default user
USER 1000
