#/bin/bash
source /root/catkin_ws/devel/setup.bash
cd /root/catkin_ws/src/
git clone $1

sudo apt-get update && rosdep update --rosdistro $ROS_DISTRO && rosdep install -y -i -r --from-path /root/catkin_ws/src
cd /root/catkin_ws
catkin_make

# A BETTER OPTION IS GIVE THE INDEX FILE AS INPUT
#echo "" > /root/.haros/index.yaml
#echo -en $2 | while IFS= read -r line ; do echo $line >> /root/.haros/index.yaml ; done 

haros full --server-host 0.0.0.0:4000
