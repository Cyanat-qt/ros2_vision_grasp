ubuntu@ubuntu-virtual-machine:~/franka_project_ros2/scripts$ bash run_demo.sh 
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2026-10-04-15-06-06-819063-ubuntu-virtual-machine-18610
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [mujoco_bridge-1]: process started with pid [18632]
[INFO] [robot_state_publisher-2]: process started with pid [18634]
[INFO] [move_group-3]: process started with pid [18636]
[INFO] [rviz2-4]: process started with pid [18638]
[INFO] [color_detector-5]: process started with pid [18640]
[INFO] [pick-6]: process started with pid [18642]
[INFO] [camera_view-7]: process started with pid [18644]
[robot_state_publisher-2] [INFO] [1791097567.557578550] [robot_state_publisher]: got segment base
[robot_state_publisher-2] [INFO] [1791097567.557845695] [robot_state_publisher]: got segment fr3_hand
[robot_state_publisher-2] [INFO] [1791097567.557857627] [robot_state_publisher]: got segment fr3_hand_tcp
[robot_state_publisher-2] [INFO] [1791097567.557862847] [robot_state_publisher]: got segment fr3_leftfinger
[robot_state_publisher-2] [INFO] [1791097567.557867797] [robot_state_publisher]: got segment fr3_link0
[robot_state_publisher-2] [INFO] [1791097567.557872796] [robot_state_publisher]: got segment fr3_link0_accelerometer_bottom
[robot_state_publisher-2] [INFO] [1791097567.557877805] [robot_state_publisher]: got segment fr3_link0_accelerometer_top
[robot_state_publisher-2] [INFO] [1791097567.557909626] [robot_state_publisher]: got segment fr3_link1
[robot_state_publisher-2] [INFO] [1791097567.557917420] [robot_state_publisher]: got segment fr3_link1_accelerometer_bottom
[robot_state_publisher-2] [INFO] [1791097567.557922781] [robot_state_publisher]: got segment fr3_link1_accelerometer_top
[robot_state_publisher-2] [INFO] [1791097567.557927610] [robot_state_publisher]: got segment fr3_link2
[robot_state_publisher-2] [INFO] [1791097567.557932749] [robot_state_publisher]: got segment fr3_link2_accelerometer_bottom
[robot_state_publisher-2] [INFO] [1791097567.557937508] [robot_state_publisher]: got segment fr3_link2_accelerometer_top
[robot_state_publisher-2] [INFO] [1791097567.557942618] [robot_state_publisher]: got segment fr3_link3
[robot_state_publisher-2] [INFO] [1791097567.557947427] [robot_state_publisher]: got segment fr3_link3_accelerometer_bottom
[robot_state_publisher-2] [INFO] [1791097567.557952276] [robot_state_publisher]: got segment fr3_link3_accelerometer_top
[robot_state_publisher-2] [INFO] [1791097567.557956985] [robot_state_publisher]: got segment fr3_link4
[robot_state_publisher-2] [INFO] [1791097567.557961684] [robot_state_publisher]: got segment fr3_link4_accelerometer_bottom
[robot_state_publisher-2] [INFO] [1791097567.557966794] [robot_state_publisher]: got segment fr3_link4_accelerometer_top
[robot_state_publisher-2] [INFO] [1791097567.557971493] [robot_state_publisher]: got segment fr3_link5
[robot_state_publisher-2] [INFO] [1791097567.557976212] [robot_state_publisher]: got segment fr3_link5_accelerometer_bottom
[robot_state_publisher-2] [INFO] [1791097567.557980981] [robot_state_publisher]: got segment fr3_link5_accelerometer_top
[robot_state_publisher-2] [INFO] [1791097567.557985629] [robot_state_publisher]: got segment fr3_link6
[robot_state_publisher-2] [INFO] [1791097567.557990358] [robot_state_publisher]: got segment fr3_link7
[robot_state_publisher-2] [INFO] [1791097567.557995087] [robot_state_publisher]: got segment fr3_link8
[robot_state_publisher-2] [INFO] [1791097567.557999836] [robot_state_publisher]: got segment fr3_rightfinger
[move_group-3] [INFO] [1791097567.809657907] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[move_group-3] [INFO] [1791097567.809795617] [moveit_robot_model.robot_model]: Loading robot model 'fr3'...
[move_group-3] [INFO] [1791097567.956534833] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-3] [INFO] [1791097567.956846853] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-3] [INFO] [1791097567.960481987] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-3] [INFO] [1791097567.964664226] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[move_group-3] [INFO] [1791097567.964822575] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-3] [INFO] [1791097567.969344891] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[move_group-3] [INFO] [1791097567.969384265] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-3] [INFO] [1791097567.969894680] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-3] [INFO] [1791097567.970428800] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-3] [WARN] [1791097567.979095970] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-3] [ERROR] [1791097567.979154029] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-3] [INFO] [1791097567.996742373] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-3] [INFO] [1791097568.140466317] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-3] [INFO] [1791097568.191238014] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-3] [INFO] [1791097568.191531779] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-3] [INFO] [1791097568.191545866] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-3] [INFO] [1791097568.191584970] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-3] [INFO] [1791097568.191611430] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.050000
[move_group-3] [INFO] [1791097568.191620788] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-3] [INFO] [1791097568.191641216] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-3] [INFO] [1791097568.191649412] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was not set. Using default value: 0.020000
[move_group-3] [INFO] [1791097568.191802421] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-3] [INFO] [1791097568.191841054] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-3] [INFO] [1791097568.191877162] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-3] [INFO] [1791097568.191887472] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-3] [INFO] [1791097568.191891149] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-3] [INFO] [1791097568.191894776] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-3] [INFO] [1791097568.616112568] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for fr3_arm_controller
[move_group-3] [INFO] [1791097568.616242624] [moveit.plugins.moveit_simple_controller_manager]: Max effort set to 0.0
[move_group-3] [INFO] [1791097568.659881152] [moveit.plugins.moveit_simple_controller_manager]: Added GripperCommand controller for franka_gripper
[move_group-3] [INFO] [1791097568.661751389] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097568.665246297] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097568.666117325] [moveit_ros.trajectory_execution_manager]: Trajectory execution is not managing controllers
[move_group-3] [INFO] [1791097568.666166758] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-3] [INFO] [1791097568.957356061] [move_group.move_group]: 
[move_group-3] 
[move_group-3] ********************************************************
[move_group-3] * MoveGroup using: 
[move_group-3] *     - ApplyPlanningSceneService
[move_group-3] *     - ClearOctomapService
[move_group-3] *     - CartesianPathService
[move_group-3] *     - ExecuteTrajectoryAction
[move_group-3] *     - GetPlanningSceneService
[move_group-3] *     - KinematicsService
[move_group-3] *     - MoveAction
[move_group-3] *     - MotionPlanService
[move_group-3] *     - QueryPlannersService
[move_group-3] *     - StateValidationService
[move_group-3] ********************************************************
[move_group-3] 
[move_group-3] [INFO] [1791097568.957492850] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-3] [INFO] [1791097568.957507598] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-3] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-3] Loading 'move_group/ClearOctomapService'...
[move_group-3] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-3] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-3] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-3] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-3] Loading 'move_group/MoveGroupMoveAction'...
[move_group-3] Loading 'move_group/MoveGroupPlanService'...
[move_group-3] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-3] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-3] 
[move_group-3] You can start planning now!
[move_group-3] 
[rviz2-4] [INFO] [1791097571.717086860] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-4] [INFO] [1791097571.717920437] [rviz2]: OpenGl version: 4.3 (GLSL 4.3)
[INFO] [camera_view-7]: process has finished cleanly [pid 18644]
[pick-6] [INFO] [1791097572.485612746] [fr3_pick_place]: READY: Waiting for stable detections and /pick/start
[rviz2-4] [INFO] [1791097572.653136926] [rviz2]: Stereo is NOT SUPPORTED
[color_detector-5] [INFO] [1791097573.089280598] [fr3_color_detector]: Color detector ready; output /perception/annotated_image and /perception/<color>/pose
[rviz2-4] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-4]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-4] [ERROR] [1791097576.459455720] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-4] [INFO] [1791097576.710550360] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-4] [INFO] [1791097580.106397721] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0 seconds
[rviz2-4] [INFO] [1791097580.106473194] [moveit_robot_model.robot_model]: Loading robot model 'fr3'...
[mujoco_bridge-1] [INFO] [1791097580.171270769] [fr3_mujoco_bridge]: MuJoCo bridge ready: RGB-D, joint states, arm and gripper actions
[rviz2-4] [INFO] [1791097580.298527427] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[rviz2-4] [INFO] [1791097580.329301214] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-4] [INFO] [1791097580.333007994] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-4] [INFO] [1791097580.359969993] [interactive_marker_display_97714566513344]: Connected on namespace: /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-4] [INFO] [1791097580.388523026] [moveit_ros_visualization.motion_planning_frame]: group fr3_arm
[rviz2-4] [INFO] [1791097580.388569364] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'fr3_arm' in namespace ''
[rviz2-4] [INFO] [1791097580.422412039] [interactive_marker_display_97714566513344]: Sending request for interactive markers
[rviz2-4] [INFO] [1791097580.473374628] [interactive_marker_display_97714566513344]: Service response received for initialization
[rviz2-4] [INFO] [1791097580.477523363] [move_group_interface]: Ready to take commands for planning group fr3_arm.
[pick-6] [INFO] [1791097595.747536556] [fr3_pick_place]: DETECTING: Waiting for stable RGB-D target
[pick-6] [INFO] [1791097595.874954557] [fr3_pick_place]: OPENING: Opening gripper
[pick-6] [INFO] [1791097596.404990376] [fr3_pick_place]: APPROACHING: Planning above visual target [0.45, 0.0, 0.0649]
[move_group-3] [INFO] [1791097596.424377055] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-3] [INFO] [1791097596.424913079] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[move_group-3] [INFO] [1791097596.425138706] [moveit_move_group_default_capabilities.move_action_capability]: Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.
[move_group-3] [WARN] [1791097596.425197377] [moveit_move_group_capabilities_base.move_group_capability]: Execution of motions should always start at the robot's current state. Ignoring the state supplied as difference in the planning scene diff
[move_group-3] [INFO] [1791097596.425214569] [moveit_ros.plan_execution]: Planning attempt 1 of at most 1
[move_group-3] [INFO] [1791097596.425224668] [moveit_move_group_capabilities_base.move_group_capability]: Using planning pipeline 'ompl'
[move_group-3] [INFO] [1791097596.430265095] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'fr3_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-3] [INFO] [1791097596.516186623] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097596.516472804] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097596.516611165] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.03
[move_group-3] [INFO] [1791097596.516850818] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-3] [INFO] [1791097596.517101382] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097596.517147980] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097596.517862621] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to fr3_arm_controller
[move_group-3] [INFO] [1791097596.588094089] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: fr3_arm_controller started execution
[move_group-3] [INFO] [1791097596.588727697] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-3] [INFO] [1791097609.430061570] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'fr3_arm_controller' successfully finished
[move_group-3] [INFO] [1791097609.437552145] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-3] [INFO] [1791097609.441617342] [moveit_move_group_default_capabilities.move_action_capability]: Solution was found and executed.
[pick-6] [INFO] [1791097609.521846335] [fr3_pick_place]: DESCENDING: Planning to grasp centre with downward tool orientation
[move_group-3] [INFO] [1791097609.524182323] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Received request to compute Cartesian path
[move_group-3] [INFO] [1791097609.524535681] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Attempting to follow 1 waypoints for link 'fr3_hand_tcp' using a step of 0.005000 m and jump threshold 2.000000 (in global reference frame)
[move_group-3] [INFO] [1791097609.570657649] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Computed Cartesian path with 26 points (followed 100.000000% of requested trajectory)
[move_group-3] [INFO] [1791097609.594591963] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-3] [INFO] [1791097609.596897904] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-3] [INFO] [1791097609.596985349] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097609.597036015] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097609.604171264] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.03
[move_group-3] [INFO] [1791097609.604794422] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-3] [INFO] [1791097609.604925820] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097609.604976095] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097609.605653105] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to fr3_arm_controller
[move_group-3] [INFO] [1791097609.663716584] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: fr3_arm_controller started execution
[move_group-3] [INFO] [1791097609.663831101] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-3] [INFO] [1791097624.100660687] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'fr3_arm_controller' successfully finished
[move_group-3] [INFO] [1791097624.107787779] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-3] [INFO] [1791097624.109248031] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: SUCCEEDED
[pick-6] [INFO] [1791097624.129192056] [fr3_pick_place]: CLOSING: Closing until contact
[pick-6] [INFO] [1791097627.132422618] [fr3_pick_place]: LIFTING: Planning upward motion with attached collision object
[move_group-3] [INFO] [1791097627.138539226] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Received request to compute Cartesian path
[move_group-3] [INFO] [1791097627.138760915] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Attempting to follow 1 waypoints for link 'fr3_hand_tcp' using a step of 0.005000 m and jump threshold 2.000000 (in global reference frame)
[move_group-3] [INFO] [1791097627.158150050] [moveit_move_group_default_capabilities.cartersian_path_service_capability]: Computed Cartesian path with 21 points (followed 100.000000% of requested trajectory)
[move_group-3] [INFO] [1791097627.187018151] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-3] [INFO] [1791097627.187615240] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-3] [INFO] [1791097627.187770353] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097627.187889509] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097627.188180319] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.03
[move_group-3] [INFO] [1791097627.189232749] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-3] [INFO] [1791097627.189492670] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097627.189567863] [moveit.plugins.moveit_simple_controller_manager]: Returned 2 controllers in list
[move_group-3] [INFO] [1791097627.190561422] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: sending trajectory to fr3_arm_controller
[move_group-3] [INFO] [1791097627.233917269] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: fr3_arm_controller started execution
[move_group-3] [INFO] [1791097627.234237514] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Goal request accepted!
[move_group-3] [INFO] [1791097639.479457631] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Controller 'fr3_arm_controller' successfully finished
[move_group-3] [INFO] [1791097639.480539627] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status SUCCEEDED ...
[move_group-3] [INFO] [1791097639.485655045] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: SUCCEEDED
[pick-6] [INFO] [1791097640.038838483] [fr3_pick_place]: SUCCEEDED: Object lifted and held; /pick/reset starts a new trial
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rviz2-4] [INFO] [1791097647.823715764] [rclcpp]: signal_handler(signum=2)
[robot_state_publisher-2] [INFO] [1791097647.823755023] [rclcpp]: signal_handler(signum=2)
[move_group-3] [INFO] [1791097647.828970751] [rclcpp]: signal_handler(signum=2)
[rviz2-4] [WARN] [1791097647.899145232] [interactive_marker_display_97714566513344]: Server not available while running, resetting
[mujoco_bridge-1] [ERROR] [1791097647.932047182] [fr3_mujoco_bridge]: RGB-D renderer failed: Failed to publish: publisher's context is invalid, at ./src/rcl/publisher.c:389
[mujoco_bridge-1] Failed to publish log message to rosout: publisher's context is invalid, at ./src/rcl/publisher.c:389
[move_group-3] [INFO] [1791097648.033320347] [moveit.ros_planning_interface.moveit_cpp]: Deleting MoveItCpp
[move_group-3] [INFO] [1791097648.085797415] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopped publishing maintained planning scene.
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 18634]
[rviz2-4] 
[rviz2-4] >>> [rcutils|error_handling.c:108] rcutils_set_error_state()
[rviz2-4] This error state is being overwritten:
[rviz2-4] 
[rviz2-4]   'rcl node's context is invalid, at ./src/rcl/node.c:428'
[rviz2-4] 
[rviz2-4] with this new error message:
[rviz2-4] 
[rviz2-4]   'publisher's context is invalid, at ./src/rcl/publisher.c:389'
[rviz2-4] 
[rviz2-4] rcutils_reset_error() should be called after error handling to avoid this.
[rviz2-4] <<<
[rviz2-4] [INFO] [1791097648.097062122] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopped publishing maintained planning scene.
[rviz2-4] [INFO] [1791097648.097307827] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[move_group-3] [INFO] [1791097648.089584987] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[move_group-3] [INFO] [1791097648.096446594] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[move_group-3] Warning: class_loader.ClassLoader: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.
[move_group-3]          at line 127 in ./src/class_loader.cpp
[INFO] [move_group-3]: process has finished cleanly [pid 18636]
[move_group-3] 
[INFO] [rviz2-4]: process has finished cleanly [pid 18638]
[INFO] [color_detector-5]: process has finished cleanly [pid 18640]
[INFO] [pick-6]: process has finished cleanly [pid 18642]
[INFO] [mujoco_bridge-1]: process has finished cleanly [pid 18632]
ubuntu@ubuntu-virtual-machine:~/franka_project_ros2/scripts$ 


