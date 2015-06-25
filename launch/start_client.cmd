set ROS_MASTER_URI=http://192.168.200.11:11311
set ROS_IP=192.168.200.5
set ROS_HOSTNAME=192.168.200.5

call C:\work\catkin_workspace\devel\setup.bat

start rosrun timesync_tester timesync_client
