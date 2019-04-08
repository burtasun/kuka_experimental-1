LANZAMIENTO RSI
roslaunch kuka_kr3_moveit kuka_kr3_moveit_planning_execution.launch sim:=false robot_ip:=172.31.100.200

	Cambio prioridad proceso kuka_hardware_interface
		renice -20 -p n_proceso

	Cambio a kernel low-latency