cd ${HOME}
source robot_ws/devel/setup.bash

####### build secondD
# mkdir /home/qh/robot_ws/map/2021-08-30-18-06-30L/recogD
# rosrun topo_handle recallD /home/qh/robot_ws/map/2021-08-30-18-06-30L 6 false

####### build DQD
# mkdir /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/recogD
# rosrun topo_handle recallD /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL 6 false

####### build second S
# mkdir /home/qh/robot_ws/map/2021-08-30-18-06-30L/recogS
# rosrun topo_handle recallS /home/qh/robot_ws/map/2021-08-30-18-06-30L 6 false

####### build DQS
# mkdir /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL/recogS
# rosrun topo_handle recallS /home/qh/robot_ws/map/2021-01-19-11-10-53DaQuanL 6 false


######3 recall o1
mkdir /home/qh/robot_ws/map/o1/recogD
rosrun topo_handle recallD /home/qh/robot_ws/map/o1 6 false

mkdir /home/qh/robot_ws/map/o1/recogS
rosrun topo_handle recallS /home/qh/robot_ws/map/o1 6 false


