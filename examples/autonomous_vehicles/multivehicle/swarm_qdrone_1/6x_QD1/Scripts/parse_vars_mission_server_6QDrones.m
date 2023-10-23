%% Auto-generated script file for parsing the stored variables
%
function parse_vars( input_data )
%

	%% Default Model Time vector - always in the first row of the data file
	assignin('base','t',input_data(1,:));

	%% Initial Home POSE (x,y,z,heading) for Vehicle 1
	assignin('base','pose_home_1',input_data(2:5,:));

	%% Initial Home POSE (x,y,z,heading) for Vehicle 2
	assignin('base','pose_home_2',input_data(6:9,:));

	%% Initial Home POSE (x,y,z,heading) for Vehicle 3
	assignin('base','pose_home_3',input_data(10:13,:));

	%% Initial Home POSE (x,y,z,heading) for Vehicle 4
	assignin('base','pose_home_4',input_data(14:17,:));

	%% Initial Home POSE (x,y,z,heading) for Vehicle 5
	assignin('base','pose_home_5',input_data(18:21,:));

	%% Initial Home POSE (x,y,z,heading) for Vehicle 6
	assignin('base','pose_home_6',input_data(22:25,:));

	%% Desired Trajectory (x,y,z,heading) for Vehicle 1
	assignin('base','traj_1',input_data(26:29,:));

	%% Desired Trajectory (x,y,z,heading) for Vehicle 2
	assignin('base','traj_2',input_data(30:33,:));

	%% Desired Trajectory (x,y,z,heading) for Vehicle 3
	assignin('base','traj_3',input_data(34:37,:));

	%% Desired Trajectory (x,y,z,heading) for Vehicle 4
	assignin('base','traj_4',input_data(38:41,:));

	%% Desired Trajectory (x,y,z,heading) for Vehicle 5
	assignin('base','traj_5',input_data(42:45,:));

	%% Desired Trajectory (x,y,z,heading) for Vehicle 6
	assignin('base','traj_6',input_data(46:49,:));

	%% Enable Signal for Swarm Trajectory (for all of the vehicles)
	assignin('base','traj_enable',input_data(50,:));

	%% Play/Pause Signal (1/0) for Swarm Trajectory (for all of the vehicles)
	assignin('base','traj_play',input_data(51,:));

	%% Loop Back Time (s) for each of the vehicles in the Swarm
	assignin('base','loop_times',input_data(52:57,:));

	%% Communiation Issue Flag for each of the vehicles in the Swarm
	assignin('base','comm_issues',input_data(58:63,:));

	%% Arm/Disarm (1/0) Signal from Operator
	assignin('base','arm',input_data(64,:));

	%% Takeoff/Land (1/0) Signal from Operator
	assignin('base','takeoff',input_data(65,:));

	%% Emergency Stop Signal (eStop) from Operator
	assignin('base','estop',input_data(66,:));

	%% Joystick Issue Signal
	assignin('base','joystick_issue',input_data(67,:));

	%% Measured Position (x,y,z) (m) for Vehicle 1
	assignin('base','pos_1',input_data(68:70,:));

	%% Measured Position (x,y,z) (m) for Vehicle 2
	assignin('base','pos_2',input_data(71:73,:));

	%% Measured Position (x,y,z) (m) for Vehicle 3
	assignin('base','pos_3',input_data(74:76,:));

	%% Measured Position (x,y,z) (m) for Vehicle 4
	assignin('base','pos_4',input_data(77:79,:));

	%% Measured Position (x,y,z) (m) for Vehicle 5
	assignin('base','pos_5',input_data(80:82,:));

	%% Measured Position (x,y,z) (m) for Vehicle 6
	assignin('base','pos_6',input_data(83:85,:));

	%% Measured Orientation (Roll, Pitch, Yaw) (rad) for Vehicle 1
	assignin('base','rot_1',input_data(86:88,:));

	%% Measured Orientation (Roll, Pitch, Yaw) (rad) for Vehicle 2
	assignin('base','rot_2',input_data(89:91,:));

	%% Measured Orientation (Roll, Pitch, Yaw) (rad) for Vehicle 3
	assignin('base','rot_3',input_data(92:94,:));

	%% Measured Orientation (Roll, Pitch, Yaw) (rad) for Vehicle 4
	assignin('base','rot_4',input_data(95:97,:));

	%% Measured Orientation (Roll, Pitch, Yaw) (rad) for Vehicle 5
	assignin('base','rot_5',input_data(98:100,:));

	%% Measured Orientation (Roll, Pitch, Yaw) (rad) for Vehicle 6
	assignin('base','rot_6',input_data(101:103,:));

	%% isTracking signal for each of the vehicles in the swarm
	assignin('base','is_tracking',input_data(104:109,:));


end