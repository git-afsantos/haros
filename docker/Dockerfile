# Use an official Python runtime as a parent image
FROM ros:kinetic-ros-base

RUN apt-get update && apt-get install -y \
    cppcheck \
    cccc \
    clang-3.8 \
    libclang-3.8-dev \
    python-pip \
    llvm-3.8-dev \
    wget

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-3.8/lib

RUN pip install --upgrade pip
RUN pip install -Iv clang==3.8
RUN pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip install haros
RUN pip install bonsai-code

RUN apt-get update && apt-get install -y ros-kinetic-desktop

SHELL ["bash", "-c"]

RUN mkdir -p ~/catkin_ws/src

# Set the working directory
WORKDIR .

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-3.8/bin/clang++

RUN source /opt/ros/kinetic/setup.bash;\
 cd ~/catkin_ws/src;\
 catkin_init_workspace;\
 cd ~/catkin_ws;\
 catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1;\
 source ~/catkin_ws/devel/setup.bash; \
 haros init

COPY ./haros_call.sh /

EXPOSE 4000

CMD []

