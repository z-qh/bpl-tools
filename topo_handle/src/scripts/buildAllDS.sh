#cd ${HOME}
#source robot_ws/devel/setup.bash

####### build secondD
#mkdir /home/qh/robot_ws/map/2021-08-30-18-06-30L/buildD
#rosrun topo_handle buildD /home/qh/robot_ws/map/2021-08-30-18-06-30L 6 false

####### build DQD
#mkdir /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/buildD
#rosrun topo_handle buildD /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL 6 false

####### build second S
#mkdir /home/qh/robot_ws/map/2021-08-30-18-06-30L/buildS
#rosrun topo_handle buildS /home/qh/robot_ws/map/2021-08-30-18-06-30L 6 false

####### build DQS
#mkdir /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/buildS
#rosrun topo_handle buildS /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL 6 false


####### oxford
mkdir /home/qh/robot_ws/map/o1/buildD
rosrun topo_handle buildD /home/qh/robot_ws/map/o1 6 false

mkdir /home/qh/robot_ws/map/o1/buildS
rosrun topo_handle buildS /home/qh/robot_ws/map/o1 6 false

