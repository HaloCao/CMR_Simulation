% control only for offset error, no trajectory generator
function exp = exp_tracking_ctrl_1point(varargin) 
    % Instantiate an empty experiment
    exp = experiment_base('tracking_control');
    
    % Add an environment map (serves as a background image)
    exp.environment = grp_obstacles_and_landmarks_from_image('../maps/emptyroom.png', 'scale', 0.01);
    
    % Some configuration options for the experiment
    poseProvider = 'ekf'; %use 'exact','ekf' or 'slam'
    
    % define the segment
    segmentPnt = [1, 0.5, degtorad(0)];
    
     % The first path point is used as initial pose
    exp.robot.initialPose = segmentPnt + [-0.8, 0.3, degtorad(15)];
    
    % instantiate the platform (differential drive) and set its parameters
    exp.robot.platform = model_platform2d_ddrive();
    exp.robot.radius = 0.14;
    exp.robot.color = [0 0 1];
    exp.robot.wheelRadius = [0.03 0.03];        % rotation and translation will be done one after another
        
    exp.robot.wheelDistance = 0.25; 
    
    % add the controller (which uses the 'platform' as input by default)
    exp.robot.controller = controller_ddrive_Lyapunov1(segmentPnt);  
    exp.robot.controller.K_x = 0.4;
    exp.robot.controller.K_phi = 3;
    
    % add sensors
    exp.robot.sensors.rangefinder = sensor_rangefinder2d();
    exp.robot.maxRange = 3;
    
    exp.robot.sensors.odometer = sensor_odometer_wheelspeed();
    exp.robot.odometryError = 10 * pi / 180;
    
    exp.robot.sensors.landmark_detector = sensor_landmarks2d();    
    exp.robot.sensors.landmark_detector.fieldOfView = 70 * pi / 180 * [-1 1];
    exp.robot.bearingError = 1 * pi / 180;
    exp.robot.rangeError = 1 / 100;
    
    switch poseProvider
    case 'slam'    
        % localization with EKF-SLAM
        exp.robot.slam = filter_ddrive_ekfslam();
        exp.robot.slam.timing = exp.robot.controller.timing;
        % By default, the SLAM filter is triggered by the landmark
        % detector, which is not synchronized to the rangefinder. Therefore
        % the pose information used during mapping may be out-of-date and
        % rather large mapping errors may arise (especially during turns).
        % Synchronizing the timings of the SLAM filter to the mapper will
        % reduce these errors
        %exp.robot.slam.timing = exp.robot.occupancyGrid.timing;
        exp.robot.slamPose = block_extract('slam', 'pose');
        exp.robot.controller.depends{1} = 'slamPose';
    case 'ekf'
        % localization from landmarks with EKF
        exp.robot.localization = filter_ddrive_ekf();
        exp.robot.localization.timing = exp.robot.controller.timing;
		%exp.robot.localization.useRange = false;
		%exp.robot.localization.useBearing = false;
        exp.robot.localizationPose = block_extract('localization', 'pose');
        exp.robot.controller.depends{1} = 'localizationPose';
    case 'exact'
        exp.robot.controller.depends{1} = 'platform';

    otherwise
        error('Invalid pose provider');            
    end
    
    exp.display.title = 'Kinematic Tracking Contrl';       
    exp.display.settings = {'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top', 'XLim', [0 8], 'YLim', [0 6]};
end