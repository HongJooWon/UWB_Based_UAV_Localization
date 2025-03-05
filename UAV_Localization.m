%% Drone UWB 3D Localization System
% This script implements a drone 3D localization system using UWB technology
% based on IEEE 802.15.4a/z standard with drone animation

clear all;
close all;
clc;

%% System Configuration
% UWB Configuration
uwbConfig = lrwpanHRPConfig('Mode', 'HPRF', 'Ranging', true);
uwbConfig.Channel = 5;  % Channel number (1-15)

% Anchor Configuration (known positions of UWB transceivers)
% Define anchors for 3D positioning
anchors = [
    0, 0, 0;       % Anchor 1 at origin
    10, 0, 0;      % Anchor 2 at (10,0,0) meters
    0, 10, 0;      % Anchor 3 at (0,10,0) meters
    10, 10, 0;     % Anchor 4 at (10,10,0) meters
    5, 5, 5        % Anchor 5 at (5,5,5) meters (ceiling)
];
numAnchors = size(anchors, 1);

% Simulation parameters
c = 299792458;             % Speed of light in m/s
simulationDuration = 15;   % seconds
timeStep = 0.05;           % seconds
timeVector = 0:timeStep:simulationDuration;
numTimeSteps = length(timeVector);

% Error modeling
clockDriftStd = 1e-10;     % Standard deviation of clock drift
rangingNoiseStd = 0.10;    % Standard deviation of ranging noise (meters)

%% Create Drone Trajectory
% Define a more complex drone trajectory
t = timeVector;

% Define a smooth trajectory using different frequencies
freq_xy = 0.15;
freq_z = 0.08;
phase_shift = pi/3;

% Base trajectory
x = 5 + 4*sin(2*pi*freq_xy*t);
y = 5 + 4*cos(2*pi*freq_xy*t + phase_shift);
z = 2 + 1.5*sin(2*pi*freq_z*t);

% Calculate attitude (orientation) for the drone
% Compute heading (yaw) based on velocity vector
dx = diff([x(1), x]);
dy = diff([y(1), y]);
yaw = atan2(dy, dx) * 180/pi;

% Compute roll and pitch based on acceleration
roll = zeros(size(t));
pitch = zeros(size(t));

for i = 2:length(t)-1
    % Approximate acceleration
    ax = (x(i+1) - 2*x(i) + x(i-1))/(timeStep^2);
    ay = (y(i+1) - 2*y(i) + y(i-1))/(timeStep^2);
    az = (z(i+1) - 2*z(i) + z(i-1))/(timeStep^2);
    
    % Convert acceleration to roll and pitch (simplified model)
    g = 9.81;  % gravity
    roll(i) = atan2(ay, sqrt(ax^2 + (az+g)^2)) * 180/pi;
    pitch(i) = atan2(ax, sqrt(ay^2 + (az+g)^2)) * 180/pi;
end

% Smooth the attitude angles
windowSize = 5;
roll = movmean(roll, windowSize);
pitch = movmean(pitch, windowSize);
yaw = movmean(yaw, windowSize);

% Actual positions matrix for the simulation
actualPositions = [x', y', z'];

%% Initialize Localization System
% Initialize storage for position estimates
estimatedPositions = zeros(numTimeSteps, 3);
rangeReadings = zeros(numTimeSteps, numAnchors);

% Extended Kalman Filter initialization
ekf = initializeEKF([x(1), y(1), z(1)], 0.1);

%% Main Simulation Loop
for t = 1:numTimeSteps
    % Get actual drone position
    actualTag = actualPositions(t, :);
    
    % Simulate UWB ranging measurements to each anchor
    ranges = zeros(numAnchors, 1);
    for i = 1:numAnchors
        % Calculate true distance
        trueDistance = norm(actualTag - anchors(i, :));
        
        % Add measurement errors (ranging noise + clock drift)
        clockError = randn(1) * clockDriftStd * c;
        rangingError = randn(1) * rangingNoiseStd;
        
        % Time of Flight ranging with error
        ranges(i) = trueDistance + rangingError + clockError;
    end
    rangeReadings(t, :) = ranges';
    
    % Estimate position using Extended Kalman Filter
    ekf = updateEKF(ekf, ranges, anchors);
    estimatedPositions(t, :) = ekf.state(1:3)';
end

%% Evaluate Localization Performance
% Calculate localization error
localizationErrors = sqrt(sum((estimatedPositions - actualPositions).^2, 2));
meanError = mean(localizationErrors);
maxError = max(localizationErrors);

% Display results
fprintf('UWB Localization Performance:\n');
fprintf('Mean localization error: %.2f m\n', meanError);
fprintf('Maximum localization error: %.2f m\n', maxError);

%% Visualization
% Create figure for trajectory analysis
figure('Name', 'Drone UWB 3D Localization', 'Position', [100, 100, 1000, 800]);

% Plot anchors and trajectories
subplot(2, 2, [1,3]); % 3D view in left half
plot3(anchors(:,1), anchors(:,2), anchors(:,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold on;

% Plot actual trajectory
plot3(actualPositions(:,1), actualPositions(:,2), actualPositions(:,3), 'g-', 'LineWidth', 2);

% Plot estimated trajectory
plot3(estimatedPositions(:,1), estimatedPositions(:,2), estimatedPositions(:,3), 'b--', 'LineWidth', 1.5);

% Add labels and legend
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D UWB Drone Localization');
grid on;
legend('Anchors', 'Actual Trajectory', 'Estimated Trajectory');
axis equal;
view(45, 30);

% Plot localization error over time
subplot(2, 2, 2);
plot(timeVector, localizationErrors, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Error (m)');
title('Localization Error vs Time');
grid on;

% Plot ranging measurements
subplot(2, 2, 4);
plot(timeVector, rangeReadings, 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Range (m)');
title('UWB Range Measurements');
grid on;
legend('Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4', 'Anchor 5');

%% Animate Drone with UWB Localization
disp('Starting drone animation with UWB localization...');

% Visualize drone movement with estimated vs actual position
% This calls the modified drone_Animation function that shows both
% actual and estimated positions
animate_DroneUWB(actualPositions(:,1), actualPositions(:,2), actualPositions(:,3), ...
                 estimatedPositions(:,1), estimatedPositions(:,2), estimatedPositions(:,3), ...
                 roll, pitch, yaw, anchors);

%% Helper Functions

% Initialize Extended Kalman Filter
function ekf = initializeEKF(initialPosition, initialUncertainty)
    % State vector: [x, y, z, vx, vy, vz]
    ekf.state = [initialPosition, 0, 0, 0]';
    
    % Initial state covariance matrix
    ekf.P = diag([initialUncertainty, initialUncertainty, initialUncertainty, ...
                  initialUncertainty, initialUncertainty, initialUncertainty]);
    
    % Process noise covariance
    ekf.Q = diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1]);
    
    % Measurement noise covariance
    ekf.R = eye(1); % Will be resized based on number of anchors
end

% Update EKF with new measurements
function ekf = updateEKF(ekf, ranges, anchors)
    % Number of anchors
    numAnchors = size(anchors, 1);
    
    % Resize measurement noise covariance if needed
    if size(ekf.R, 1) ~= numAnchors
        ekf.R = 0.1 * eye(numAnchors);
    end
    
    % Prediction step (simple constant velocity model)
    dt = 0.05; % Time step
    F = [1 0 0 dt 0 0;
         0 1 0 0 dt 0;
         0 0 1 0 0 dt;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
    
    % Predict state and covariance
    ekf.state = F * ekf.state;
    ekf.P = F * ekf.P * F' + ekf.Q;
    
    % Update step
    % Calculate expected ranges based on current state estimate
    expectedRanges = zeros(numAnchors, 1);
    H = zeros(numAnchors, 6); % Jacobian matrix
    
    for i = 1:numAnchors
        % Current position estimate
        pos = ekf.state(1:3);
        
        % Vector from tag to anchor
        delta = pos - anchors(i, :)';
        
        % Expected range to this anchor
        expectedRanges(i) = norm(delta);
        
        % Jacobian of measurement model
        if expectedRanges(i) > 0
            H(i, 1:3) = delta' / expectedRanges(i);
        end
    end
    
    % Kalman gain
    S = H * ekf.P * H' + ekf.R;
    K = ekf.P * H' / S;
    
    % Update state and covariance
    ekf.state = ekf.state + K * (ranges - expectedRanges);
    ekf.P = (eye(6) - K * H) * ekf.P;
end

% Animation function for drone with UWB localization
function animate_DroneUWB(x_actual, y_actual, z_actual, x_est, y_est, z_est, roll, pitch, yaw, anchors)
    % This Animation code is for QuadCopter with UWB localization
    % Modified from Jitendra Singh's original code
    
    %% Define design parameters
    D2R = pi/180;
    R2D = 180/pi;
    b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
    a   = b/3;   % the legth of small square base of quadcopter(b/4)
    H   = 0.06;  % hight of drone in Z direction
    H_m = H+H/2; % hight of motor in z direction
    r_p = b/4;   % radius of propeller
    
    %% Conversions
    ro = 45*D2R;                   % angle by which rotate the base of quadcopter
    Ri = [cos(ro) -sin(ro) 0;
          sin(ro) cos(ro)  0;
           0       0       1];     % rotation matrix to rotate the coordinates of base 
    base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
               -a/2 -a/2 a/2 a/2;
                 0    0   0   0];
    base = Ri*base_co;             % rotate base Coordinates by 45 degree 

    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1,length(to));
    
    %% Define Figure plot
    fig1 = figure('pos', [50 50 1000 800], 'Name', 'Drone UWB Localization Animation');
    hg = gca;
    view(45, 30);
    grid on;
    axis equal;
    
    % Set axis limits based on the drone trajectory
    xlim([min([x_actual; x_est; anchors(:,1)])-1.5, max([x_actual; x_est; anchors(:,1)])+1.5]); 
    ylim([min([y_actual; y_est; anchors(:,2)])-1.5, max([y_actual; y_est; anchors(:,2)])+1.5]); 
    zlim([min([z_actual; z_est; anchors(:,3)]), max([z_actual; z_est; anchors(:,3)])+1.5]);
    
    title('Drone UWB Localization Animation')
    xlabel('X[m]');
    ylabel('Y[m]');
    zlabel('Z[m]');
    hold(hg, 'on');
    
    % Plot anchors
    for i = 1:size(anchors, 1)
        plot3(anchors(i,1), anchors(i,2), anchors(i,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        text(anchors(i,1)+0.2, anchors(i,2)+0.2, anchors(i,3)+0.2, ['Anchor ', num2str(i)]);
    end
    
    %% Design Different parts
    % design the base square
    drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
    drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
    alpha(drone(1:2),0.7);
    
    % design 2 perpendicular legs of quadcopter 
    [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
    drone(3) = surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
    drone(4) = surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b'); 
    alpha(drone(3:4),0.6);
    
    % design 4 cylindrical motors 
    drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
    drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
    drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
    drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
    alpha(drone(5:8),0.7);
    
    % design 4 propellers
    drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
    drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
    drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'m','LineWidth',0.5);
    drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'m','LineWidth',0.5);
    alpha(drone(9:12),0.3);

    %% Create a group object and parent surface
    combinedobject = hgtransform('parent', hg);
    set(drone, 'parent', combinedobject);
 
    %% Animation loop
    % Initialize trajectory plots
    h_actual = plot3(NaN, NaN, NaN, 'g-', 'LineWidth', 2);
    h_est = plot3(NaN, NaN, NaN, 'b--', 'LineWidth', 1.5);
    
    % Display initial position markers
    plot3(x_actual(1), y_actual(1), z_actual(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(x_est(1), y_est(1), z_est(1), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Add legend
    legend('Anchors', 'Actual Trajectory', 'Estimated Trajectory', 'Drone');
    
    % Animation loop
    for i = 1:length(x_actual)
        % Update trajectory plots
        set(h_actual, 'XData', x_actual(1:i), 'YData', y_actual(1:i), 'ZData', z_actual(1:i));
        set(h_est, 'XData', x_est(1:i), 'YData', y_est(1:i), 'ZData', z_est(1:i));
        
        % Compute current error
        current_error = norm([x_actual(i), y_actual(i), z_actual(i)] - [x_est(i), y_est(i), z_est(i)]);
        
        % Update title with current error
        title(sprintf('Drone UWB Localization Animation - Current Error: %.2f m', current_error));
       
        % Transform drone to actual position
        translation = makehgtform('translate', [x_actual(i), y_actual(i), z_actual(i)]);
        rotation1 = makehgtform('xrotate', (pi/180)*(roll(i)));
        rotation2 = makehgtform('yrotate', (pi/180)*(pitch(i)));
        rotation3 = makehgtform('zrotate', (pi/180)*yaw(i));
        set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);
        
        % Draw line from drone to estimated position
        if i > 1
            line([x_actual(i), x_est(i)], [y_actual(i), y_est(i)], [z_actual(i), z_est(i)], ...
                'Color', 'r', 'LineStyle', ':', 'LineWidth', 1);
        end
        
        drawnow;
        pause(0.02);
    end
end