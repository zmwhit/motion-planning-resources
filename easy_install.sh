#!/bin/bash
set -e
REPO_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/" && pwd )"
TMP_DIR="/tmp"
install_ros_desktop() {
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    curl -O https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc
    sudo apt-key add ros.asc
    sudo apt install ros-melodic-desktop
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    sudo rosdep init
    rosdep update
}
install_ros_deps() {
    echo "Prepare to install ros dependencies"
    sudo apt-get install ros-melodic-stage ros-melodic-map-server
}
install_g2o() {
    echo "Prepare to install g2o"
    sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
    if ( ls /usr/local/include | grep g2o);then
        echo "g2o is already installed......"
    else
        wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20201223_git.zip
        unzip 20201223_git.zip
        cd g2o-20201223_git
        mkdir build && cd build
        cmake ..
        make -j$(nproc) 
        sudo make install
        echo "g2o installed successfully"   
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_casadi() {
    echo "Prepare to install casadi"
    if ( ls /usr/local/include | grep casadi);then
        echo "casadi is already installed......"
    else
        cd $TEMP_DIR
        wget https://github.com/casadi/casadi/releases/download/3.5.5/casadi-3.5.5-1.tar.gz
        tar -zxvf casadi-3.5.5-1.tar.gz
        cd casadi-3.5.5.1
        mkdir build && cd build
        cmake .. -DWITH_IPOPT=ON -DWITH_EXAMPLES=OFF -DWITH_OSQP=ON -DWITH_QPOASES=ON -DWITH_LAPACK=ON
        make -j$(nproc) 
        sudo make install
        echo "Casadi installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_ipopt() {
    echo "Prepare to install ipopt"
    sudo apt install wget unzip gfortran liblapack-dev patch pkg-config libmetis-dev cppad
    if ( ls /usr/local/include | grep coin | grep coin-or);then
        echo "ipopt is already installed......"
    else
        git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git --depth=1
        cd ThirdParty-Mumps
        ./get.Mumps
        ./configure --prefix=/usr/local
        make -j$(nproc) 
        sudo make install
        echo "Mumps installed successfully"
        cd $REPO_DIR
        wget https://github.com/coin-or/Ipopt/archive/refs/heads/stable/3.14.zip
        unzip 3.14.zip && cd Ipopt-stable-3.14/
        ./configure --prefix=/usr/local
        make -j$(nproc) 
        sudo make install
        echo "Ipopt installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
    cd /usr/local/include
    sudo mkdir coin
    sudo cp -r ./coin-or/* ./coin
    cd $REPO_DIR
}
install_qpOASES() {
    echo "Prepare to install qpOASES"
    if ( ls /usr/local/include | grep qpOASES);then
        echo "qpOASE is already installed......"
    else
        git clone --recursive https://github.com/oxfordcontrol/osqp
        cd osqp
        mkdir build && cd build
        cmake ..
        make -j$(nproc) 
        sudo make install     
        echo "qpOASES installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_osqp_eigen() {
    sudo apt-get install libeigen3-dev
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    echo "eigen3 is installed successfully!"
    echo "Prepare to install osqp"
    if ( ls /usr/local/include | grep osqp);then
        echo "osqp is already installed......"
    else
        git clone --recursive https://github.com/oxfordcontrol/osqp
        cd osqp
        mkdir build && cd build
        cmake -G "Unix Makefiles" ..
        cmake --build .
        sudo cmake --build . --target install
        echo "osqp installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
    echo "Prepare to install osqp-eigen"
    if ( ls /usr/local/include | grep OsqpEigen);then
        echo "OsqpEigen is already installed......"
    else
        git clone https://github.com/robotology/osqp-eigen.git
        cd osqp-eigen
        mkdir build && cd build
        cmake ../
        make -j$(nproc) 
        sudo make install
        echo "osqp-eigen installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_benchmark() {
    echo "Prepare to install google benchmark"
    if ( ls /usr/local/include | grep benchmark);then
        echo "benchmark is already installed......"
    else
    
        git clone https://github.com/google/benchmark.git
        cd benchmark
        git clone https://github.com/google/googletest.git
        mkdir build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=RELEASE
        make -j$(nproc) 
        sudo make install
        echo "benchmark installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_glog() {
    sudo apt-get install libgoogle-glog-dev
    echo "glog is installed successfully!"
}
install_gflags() {
    sudo apt-get install libgflags-dev
    echo "gflags is installed successfully!"
}
install_protobuf() {
    sudo apt-get install autoconf automake libtool curl make g++ unzip libffi-dev
    echo "Prepare to install google protobuf"
    if (ls /usr/local/include | grep google);then
        echo "google protobuf is already installed......"
    else
        git clone https://github.com/google/protobuf.git
        cd protobuf
        ./autogen.sh
        ./configure
        make -j$(nproc) 
        sudo make install
        echo "google protobuf installed successfully"
        cd $REPO_DIR
    fi
    sudo ldconfig 
}
install_apollo() {
    cd ~
    git init
    git clone https://github.com/ApolloAuto/apollo
    cd ~/apollo
    bash docker/scripts/dev_start.sh -y
}
install_docker() {    
    sudo modprobe overlay
    sudo docker -v 1>/dev/null 2>&1
    if [ $? -eq 0 ]
    then
        id | grep "docker" 1>/dev/null 2>&1
        if [ $? -eq 0 ]
        then
            echo "docker is OK!"
            return 1
        else
            sudo gpasswd -a $USER docker  
            sudo usermod -aG docker $USER
            sudo systemctl restart docker
            echo "please reboot the computer and run the scripts again!"
            return 2
        fi
    else
        curl https://get.docker.com | sh && sudo systemctl --now enable docker
        sudo systemctl restart docker
        sudo gpasswd -a $USER docker  
        sudo usermod -aG docker $USER
        sudo systemctl restart docker
        sudo chmod 777 /var/run/docker.sock
        echo "please reboot the computer and run the scripts again!"
        return 3
    fi

    curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list |  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    sudo apt-get install -y nvidia-docker2
    sudo pkill -SIGHUP dockerd
    return 0
}
install_conda() {
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh
    bash miniconda.sh
}
install_prepare() {
    sudo apt-get install gcc g++ git vim curl make cmake gedit unzip cutecom can-utils net-tools
    sudo apt-get install mlocate terminator openssh-client openssh-server doxygen
}
main() {
    echo ${REPO_DIR}
    # install_prepare
    # install_docker
    # install_conda

    # install_benchmark
    # install_glog
    # install_gflags
    # install_protobuf
    
    # install_ipopt
    # install_osqp_eigen
    # install_qpOASES
    # install_casadi
    # install_g2o

    # install_ros_desktop
    # install_ros_deps

    # install_apollo
}


main