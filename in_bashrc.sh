master=172.17.0.1
client=172.17.0.2

export ROS_MASTER_URI=http://$master:11311
export ROS_HOSTNAME=$client
export ROS_IP=$client

cd ~/shape_based_matching
