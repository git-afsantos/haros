# Use an official Python runtime as a parent image
FROM ros:melodic

RUN apt-get update && apt-get install -y \
    cppcheck \
    cccc \
    clang-4.0 \
    libclang-4.0-dev \
    python-pip \
    llvm-4.0-dev \
    wget \
    netbase \
    libmagic-dev

ENV LD_LIBRARY_PATH $LD_LIBRARY_PATH:/usr/lib/llvm-4.0/lib

RUN pip install --upgrade pip
RUN pip install -Iv clang==4.0
RUN pip install -e git+https://github.com/timtadh/pyflwor.git#egg=pyflwor
RUN pip install haros
RUN pip install bonsai-code

RUN apt-get update && apt-get install -y ros-melodic-desktop

SHELL ["bash", "-c"]

RUN mkdir -p ~/catkin_ws/src

# Set the working directory
WORKDIR .

ENV CMAKE_CXX_COMPILER /usr/lib/llvm-4.0/bin/clang++

RUN source /opt/ros/melodic/setup.bash;\
 cd ~/catkin_ws/src;\
 catkin_init_workspace;\
 cd ~/catkin_ws;\
 catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1;\
 source ~/catkin_ws/devel/setup.bash; \
 haros init

COPY ./haros_call.sh /

EXPOSE 4000

CMD []

