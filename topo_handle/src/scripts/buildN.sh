cd ${HOME}
source robot_ws/devel/setup.bash

####### build second N
#mkdir /home/qh/robot_ws/map/2021-08-30-18-06-30L/buildN
#rosrun topo_handle buildN /home/qh/robot_ws/map/2021-08-30-18-06-30L 6 false 2

####### build DQN
#mkdir /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/buildN
#rosrun topo_handle buildN /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL 6 false 2


####### build o1
mkdir /home/qh/robot_ws/map/o1/buildN
rosrun topo_handle buildN /home/qh/robot_ws/map/o1 6 false 2
