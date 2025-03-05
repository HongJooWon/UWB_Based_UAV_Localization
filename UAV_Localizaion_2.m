%% Drone UWB 3D Localization Example
% This script demonstrates a drone flying in a straight line for 3m
% while UWB-based localization tracks its position

%% System Configuration
% Clear workspace and close figures
clear all;
close all;
clc;

% UWB Configuration
uwbConfig = struct();
uwbConfig.MeanPRF = 249.6;     % MHz (HPRF mode)
uwbConfig.Channel = 5;         % Channel number (1-15)
uwbConfig.RangingEnabled = true;

% Anchor Configuration (known positions of UWB transceivers)
anchors = [
    0, 0, 0;       % Anchor 1 at origin
    3, 0, 0;       % Anchor 2 at (3,0,0) meters
    0, 3, 0;       % Anchor 3 at (0,3,0) meters
    0, 0, 2;       % Anchor 4 at (0,0,2) meters
    3, 3, 1        % Anchor 5 at (3,3,1) meters
];
numAnchors = size(anchors, 1);

% Simulation parameters
c = 299792458;             % Speed of light in m/s
simulationDuration = 10;   % seconds
timeStep = 0.1;            % seconds
timeVector = 0:timeStep:simulationDuration;
numTimeSteps = length(timeVector);

% Error modeling
clockDriftStd = 1e-10;     % Standard deviation of clock drift
rangingNoiseStd = 0.05;    % Standard deviation of ranging noise (meters)

%% Drone Flight Path - Straight Line 3m
% Initial drone position
initialPosition = [0.5, 0.5, 0.5];  % Starting position (meters)

% Define a simple straight-line path for 3 meters in the X direction
endPosition = [3.5, 0.5, 0.5];      % End position (meters)

% Calculate velocity to move 3m in simulation duration
velocity = (endPosition - initialPosition) / simulationDuration;

% Generate drone trajectory
dronePositions = zeros(numTimeSteps, 3);
for t = 1:numTimeSteps
    dronePositions(t, :) = initialPosition + velocity * timeVector(t);
end

% Add a small sinusoidal motion in z-axis to simulate flight instability
dronePositions(:, 3) = dronePositions(:, 3) + 0.05 * sin(2*pi*0.5*timeVector');

%% Initialize EKF for Position Estimation
ekf = initializeEKF(initialPosition, 0.1);

%% Main Simulation Loop
% Initialize storage for position estimates
estimatedPositions = zeros(numTimeSteps, 3);
rangeReadings = zeros(numTimeSteps, numAnchors);

% Flight attitude (roll, pitch, yaw)
roll = 2 * sin(2*pi*0.2*timeVector);  % Small roll oscillation (degrees)
pitch = 2 * cos(2*pi*0.2*timeVector); % Small pitch oscillation (degrees)
yaw = zeros(size(timeVector));        % No yaw rotation (heading stays constant)

% Main loop
for t = 1:numTimeSteps
    % Current drone position
    currentPosition = dronePositions(t, :);
    
    % Simulate UWB ranging measurements to each anchor
    ranges = zeros(numAnchors, 1);
    for i = 1:numAnchors
        % Calculate true distance
        trueDistance = norm(currentPosition - anchors(i, :));
        
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
localizationErrors = sqrt(sum((estimatedPositions - dronePositions).^2, 2));
meanError = mean(localizationErrors);
maxError = max(localizationErrors);

% Display results
fprintf('Mean localization error: %.2f m\n', meanError);
fprintf('Maximum localization error: %.2f m\n', maxError);

%% Visualization
figure('Name', 'Drone UWB 3D Localization', 'Position', [100, 100, 1000, 800]);

% Plot 3D view of the environment
subplot(2, 2, [1,3]); % 3D view in left half
hold on;

% Plot anchors
plot3(anchors(:,1), anchors(:,2), anchors(:,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Plot actual drone trajectory
plot3(dronePositions(:,1), dronePositions(:,2), dronePositions(:,3), 'g-', 'LineWidth', 2);

% Plot estimated trajectory
plot3(estimatedPositions(:,1), estimatedPositions(:,2), estimatedPositions(:,3), 'b--', 'LineWidth', 1.5);

% Add labels and legend
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Drone 3D UWB Localization');
grid on;
legend('Anchors', 'Actual Trajectory', 'Estimated Trajectory');
axis equal;
view(45, 30); % Set view angle

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
legend('Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4', 'Anchor 5', 'Location', 'best');

%% Animate Drone Movement
figure('Name', 'Drone Animation', 'Position', [100, 100, 800, 600]);
view(45, 30);
grid on;
axis equal;
xlim([0 4]); ylim([0 4]); zlim([0 3]);
title('Drone Flight with UWB Localization')
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');
hold on;

% Plot anchors in animation
h_anchors = plot3(anchors(:,1), anchors(:,2), anchors(:,3), 'ro', 'MarkerSize', 10, 'DisplayName', 'Anchor');
for i = 1:numAnchors
    text(anchors(i,1), anchors(i,2), anchors(i,3), ['  Anchor ', num2str(i)], 'HandleVisibility', 'off');
end

% Call animation function
animateDrone(dronePositions(:,1), dronePositions(:,2), dronePositions(:,3), ...
             roll, pitch, yaw, estimatedPositions);

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
    dt = 0.1; % Time step
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

% Animation function for drone
function animateDrone(x, y, z, roll, pitch, yaw, estimatedPositions)
    % Define drone design parameters
    D2R = pi/180;
    b = 0.4;   % Drone body size in meters
    a = b/3;   % Base size
    H = 0.06;  % Height
    H_m = H+H/2; % Motor height
    r_p = b/4; % Propeller radius
    
    % Base rotation
    ro = 45*D2R;
    Ri = [cos(ro) -sin(ro) 0;
          sin(ro) cos(ro)  0;
           0       0       1];
           
    % Base coordinates
    base_co = [-a/2  a/2 a/2 -a/2;
               -a/2 -a/2 a/2 a/2;
                 0    0   0   0];
    base = Ri*base_co;
    
    % Propeller coordinates
    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1,length(to));
    
    % Design drone parts
    % Base square
    drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
    drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
    alpha(drone(1:2),0.7);
    
    % Legs
    [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
    drone(3) = surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
    drone(4) = surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b'); 
    alpha(drone(3:4),0.6);
    
    % Motors
    drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
    drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
    drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
    drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
    alpha(drone(5:8),0.7);
    
    % Propellers
    drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
    drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
    drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
    drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
    alpha(drone(9:12),0.3);
    
    % Create transform group
    hg = gca;
    combinedobject = hgtransform('parent', hg);
    set(drone, 'parent', combinedobject)
    
    % Set all drone parts to be invisible in legend
    for i = 1:length(drone)
        set(drone(i), 'HandleVisibility', 'off');
    end
    
    % Trajectory lines
    actualTrajectory = animatedline('Color','g','LineWidth',2,'LineStyle','-', 'DisplayName', 'Actual Trajectory');
    estimatedTrajectory = animatedline('Color','b','LineWidth',1.5,'LineStyle','--', 'DisplayName', 'Estimated Trajectory');
    
    % Plot dummy objects for correct legend
    h1 = plot3(NaN, NaN, NaN, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    h2 = plot3(NaN, NaN, NaN, 'g-', 'LineWidth', 2);
    h3 = plot3(NaN, NaN, NaN, 'b--', 'LineWidth', 1.5);
    
    % Create proper legend
    legend([h1, h2, h3], 'Anchor', 'Actual Trajectory', 'Estimated Trajectory');
    
    % Animation loop
    for i = 1:length(x)
        % Add to trajectory lines
        addpoints(actualTrajectory, x(i), y(i), z(i));
        addpoints(estimatedTrajectory, estimatedPositions(i,1), estimatedPositions(i,2), estimatedPositions(i,3));
        
        % Transform drone
        translation = makehgtform('translate', [x(i), y(i), z(i)]);
        rotation1 = makehgtform('xrotate', (pi/180)*(roll(i)));
        rotation2 = makehgtform('yrotate', (pi/180)*(pitch(i)));
        rotation3 = makehgtform('zrotate', (pi/180)*yaw(i));
        
        set(combinedobject, 'matrix', translation*rotation3*rotation2*rotation1);
        
        % Draw error line connecting actual and estimated positions
        if mod(i, 5) == 0  % Draw error line every 5 frames to avoid clutter
            line([x(i), estimatedPositions(i,1)], ...
                 [y(i), estimatedPositions(i,2)], ...
                 [z(i), estimatedPositions(i,3)], ...
                 'Color', 'k', 'LineStyle', ':', 'LineWidth', 0.5, ...
                 'HandleVisibility', 'off'); % Hide from legend
        end
        
        drawnow
        pause(0.02);
    end
end