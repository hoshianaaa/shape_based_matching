PROGRAM_PATH=$HOME/shape_based_matching
<<<<<<< HEAD
#gnome-terminal --tab -t "roscore" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; roscore; exec bash"
#sleep 1
#gnome-terminal --tab -t "image publisher" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; cd $PROGRAM_PATH; python img_pub.py search.png; exec bash"
#gnome-terminal --tab -t "image cripper" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; cd $PROGRAM_PATH; python clipper_image_ros.py; exec bash"
=======
gnome-terminal --tab -t "roscore" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; roscore; exec bash"
sleep 1
gnome-terminal --tab -t "image publisher" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; cd $PROGRAM_PATH; python img_pub.py search.png; exec bash"
gnome-terminal --tab -t "image cripper" -- bash -c "source /opt/ros/*/setup.bash; source $HOME/catkin_ws/devel/setup.bash; cd $PROGRAM_PATH; python clipper_image_ros.py; exec bash"
>>>>>>> e5f0e23e3ac06b6fc836c481e9d20be881cb80a7
gnome-terminal --tab -t "shape_based_matching" -- bash -c "cd $PROGRAM_PATH/build; ./shape_based_matching_test; exec bash"
