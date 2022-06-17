PROGRAM_PATH=$HOME/shape_based_matching
gnome-terminal --tab -t "roscore" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; roscore; exec bash"
sleep 1
gnome-terminal --tab -t "image publisher" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; cd $PROGRAM_PATH; python img_pub.py search.png; exec bash"
gnome-terminal --tab -t "image cripper" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; cd $PROGRAM_PATH; python clipper_image_ros.py; exec bash"
