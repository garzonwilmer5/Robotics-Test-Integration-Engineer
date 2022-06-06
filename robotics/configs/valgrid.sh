valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='valgrid_log/lifecycle_manager_output.txt' -v ./robotics/ros2/install/lifecycle_manager/lib/lifecycle_manager/lifecycle_manager
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='valgrid_log/motion_control_output.txt' -v ./robotics/ros2/install/motion_control/lib/motion_control/speed_controller
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='valgrid_log/rpm_converter_output.txt' -v ./robotics/ros2/install/rpm_converter/lib/rpm_converter/rpm_converter_node
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='valgrid_log/wheel_odometry_output.txt' -v ./robotics/ros2/install/wheel_odometry/lib/wheel_odometry/wheel_odometry
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='valgrid_log/interfaces_output.txt' -v ./robotics/ros2/install/interfaces/lib/interfaces/interfaces_node