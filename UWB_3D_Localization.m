%% UWB 3D Localization System using IEEE 802.15.4a/z
% This script implements 3D object localization using UWB signals
% based on IEEE 802.15.4a/z standard

%% System Configuration
% Define the environment and system parameters
clear all;
close all;

% UWB Configuration
uwbConfig = lrwpanHRPConfig('Mode', 'HPRF', 'Ranging', true);
uwbConfig.Channel = 5;  % Channel number (1-15)
% HPRF 모드는 기본적으로 249.6 MHz PRF 사용
% MeanPRF 속성을 직접 설정할 수도 있음 (대체 옵션)
% uwbConfig.MeanPRF = 249.6;

% Anchor Configuration (known positions of UWB transceivers)
% Define 4 or more anchors for 3D positioning
anchors = [
    0, 0, 0;       % Anchor 1 at origin
    10, 0, 0;      % Anchor 2 at (10,0,0) meters
    0, 10, 0;      % Anchor 3 at (0,10,0) meters
    0, 0, 3;       % Anchor 4 at (0,0,3) meters
    10, 10, 2      % Anchor 5 at (10,10,2) meters
];
numAnchors = size(anchors, 1);

% Tag initial position (to be estimated)
tagPosition = [3, 4, 1.5];  % Example position in meters

% Simulation parameters
c = 299792458;             % Speed of light in m/s
simulationDuration = 10;   % seconds
timeStep = 0.1;            % seconds
timeVector = 0:timeStep:simulationDuration;
numTimeSteps = length(timeVector);

% Error modeling
clockDriftStd = 1e-10;     % Standard deviation of clock drift
rangingNoiseStd = 0.05;    % Standard deviation of ranging noise (meters)

%% Main Simulation Loop
% Initialize storage for position estimates
estimatedPositions = zeros(numTimeSteps, 3);
actualPositions = zeros(numTimeSteps, 3);
rangeReadings = zeros(numTimeSteps, numAnchors);

% Create a simple movement pattern for the tag
velocityVector = [0.2, 0.3, 0.1];  % m/s in x, y, z

% Extended Kalman Filter initialization
ekf = initializeEKF(tagPosition, 0.1);

% Main loop
for t = 1:numTimeSteps
    % Update actual tag position with some movement
    actualTag = tagPosition + velocityVector * timeVector(t);
    actualPositions(t, :) = actualTag;
    
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
fprintf('Mean localization error: %.2f m\n', meanError);
fprintf('Maximum localization error: %.2f m\n', maxError);

%% Visualization
figure('Name', 'UWB 3D Localization');

% Plot anchors
subplot(2, 2, [1,3]); % 3D view in left half
plot3(anchors(:,1), anchors(:,2), anchors(:,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
hold on;

% Plot actual trajectory
plot3(actualPositions(:,1), actualPositions(:,2), actualPositions(:,3), 'g-', 'LineWidth', 2);

% Plot estimated trajectory
plot3(estimatedPositions(:,1), estimatedPositions(:,2), estimatedPositions(:,3), 'b--', 'LineWidth', 1.5);

% Connect each point with lines for visibility
for i = 1:numTimeSteps
    plot3([actualPositions(i,1), estimatedPositions(i,1)], ...
          [actualPositions(i,2), estimatedPositions(i,2)], ...
          [actualPositions(i,3), estimatedPositions(i,3)], 'k:');
end

% Add labels and legend
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D UWB Localization');
grid on;
legend('Anchors', 'Actual Trajectory', 'Estimated Trajectory', 'Error');
axis equal;

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

% Simulates UWB Two-Way Ranging between devices
function [distance, tof] = simulateTwoWayRanging(uwbConfig, distance, noiseStd)
    % Constants
    c = 299792458; % Speed of light in m/s
    
    % Generate random data for UWB packet
    dataLength = 1000; % bits
    data = randi([0 1], dataLength, 1);
    
    % Generate UWB waveform
    waveform = lrwpanWaveformGenerator(data, uwbConfig);
    
    % Calculate theoretical time of flight
    tofIdeal = distance / c;
    
    % Add measurement noise
    tof = tofIdeal + randn(1) * noiseStd / c;
    
    % Calculate measured distance
    distance = tof * c;
end

% Trilateration algorithm for position estimation
function position = trilateration(anchors, ranges)
    % Minimum 4 anchors needed for 3D positioning
    numAnchors = size(anchors, 1);
    
    if numAnchors < 4
        error('At least 4 anchors are required for 3D positioning');
    end
    
    % Initial position estimate (centroid of anchors)
    position = mean(anchors, 1);
    
    % Use least squares to refine the position
    options = optimoptions('lsqnonlin', 'Display', 'off');
    position = lsqnonlin(@costFunction, position, [], [], options);
    
    % Nested cost function for optimization
    function cost = costFunction(pos)
        cost = zeros(numAnchors, 1);
        for i = 1:numAnchors
            expectedRange = norm(pos - anchors(i, :));
            cost(i) = expectedRange - ranges(i);
        end
    end
end
