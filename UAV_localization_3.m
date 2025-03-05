%% Drone UWB 3D Localization with TDOA, Kalman Filter, and Particle Filter
% This script implements a TDOA-based localization system for a drone
% using both Kalman Filter for RSSI smoothing and Particle Filter for position estimation
% Based on the paper: "Particle Filtering-Based Indoor Positioning System for Beacon Tag Tracking"

%% System Configuration
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
    0, 0, 0;       % Anchor 1 at origin (reference anchor for TDOA)
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
rssiNoiseStd = 2;          % Standard deviation of RSSI noise (dBm)

% RSSI parameters - path loss model
rssiAtOneMeter = -65;      % RSSI at 1m distance (dBm)
pathLossExponent = 2.0;    % Path loss exponent

% Particle filter parameters
numParticles = 1000;       % Number of particles
effectiveParticleThreshold = 0.5 * numParticles;  % Threshold for resampling
initialUncertainty = 1.0;  % Initial position uncertainty (meters)

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

%% Initialize Kalman Filter for RSSI Smoothing and Particle Filter for Position Estimation
% Initialize KF for RSSI smoothing for each anchor
kfRSSI = initializeRSSIKalmanFilters(numAnchors);

% Initialize particles randomly around the initial position
particles = initializeParticles(numParticles, initialPosition, initialUncertainty);

%% Main Simulation Loop
% Initialize storage for measurements and position estimates
tdoaMeasurements = zeros(numTimeSteps, numAnchors-1);
rawRSSI = zeros(numTimeSteps, numAnchors);
smoothedRSSI = zeros(numTimeSteps, numAnchors);
estimatedPositions = zeros(numTimeSteps, 3);
particlePositions = cell(numTimeSteps, 1);  % Store particles for visualization

% Main loop
for t = 1:numTimeSteps
    % Current drone position
    currentPosition = dronePositions(t, :);
    
    % 1. Simulate UWB TOF measurements to each anchor
    trueDistances = zeros(numAnchors, 1);
    for i = 1:numAnchors
        trueDistances(i) = norm(currentPosition - anchors(i, :));
    end
    
    % 2. Compute TDOA measurements relative to reference anchor (anchor 1)
    trueTDOA = zeros(numAnchors-1, 1);
    noisyTDOA = zeros(numAnchors-1, 1);
    for i = 2:numAnchors
        % True TDOA is difference in distances
        trueTDOA(i-1) = trueDistances(i) - trueDistances(1);
        
        % Add measurement errors to TDOA
        tdoaError = randn(1) * rangingNoiseStd;
        noisyTDOA(i-1) = trueTDOA(i-1) + tdoaError;
    end
    tdoaMeasurements(t, :) = noisyTDOA';
    
    % 3. Generate RSSI measurements
    rssiValues = zeros(numAnchors, 1);
    for i = 1:numAnchors
        % Path loss model: RSSI = RSSI_1m - 10*n*log10(d)
        rssiValues(i) = rssiAtOneMeter - 10 * pathLossExponent * log10(trueDistances(i));
        
        % Add noise to RSSI
        rssiValues(i) = rssiValues(i) + randn(1) * rssiNoiseStd;
    end
    rawRSSI(t, :) = rssiValues';
    
    % 4. Apply Kalman Filter to smooth RSSI measurements
    smoothedRssiValues = zeros(numAnchors, 1);
    for i = 1:numAnchors
        kfRSSI{i} = updateRSSIKalmanFilter(kfRSSI{i}, rssiValues(i));
        smoothedRssiValues(i) = kfRSSI{i}.state;
    end
    smoothedRSSI(t, :) = smoothedRssiValues';
    
    % 5. Convert smoothed RSSI to estimated distances
    estimatedDistances = rssiToDistance(smoothedRssiValues, rssiAtOneMeter, pathLossExponent);
    
    % 6. Update Particle Filter with TDOA measurements
    particles = updateParticleFilter(particles, noisyTDOA, smoothedRssiValues, anchors, numParticles, effectiveParticleThreshold);
    particlePositions{t} = particles;
    
    % 7. Estimate position from particle filter
    estimatedPositions(t, :) = estimatePositionFromParticles(particles);
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
figure('Name', 'Drone UWB 3D Localization with TDOA and Particle Filter', 'Position', [100, 100, 1000, 800]);

% Plot 3D view of the environment
subplot(2, 2, [1,3]); % 3D view in left half
hold on;

% Plot anchors
plot3(anchors(:,1), anchors(:,2), anchors(:,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(anchors(1,1), anchors(1,2), anchors(1,3), '  Reference Anchor', 'Color', 'r');

% Plot actual drone trajectory
plot3(dronePositions(:,1), dronePositions(:,2), dronePositions(:,3), 'g-', 'LineWidth', 2);

% Plot estimated trajectory
plot3(estimatedPositions(:,1), estimatedPositions(:,2), estimatedPositions(:,3), 'b--', 'LineWidth', 1.5);

% Visualize particles at select times
frameIdx = round(linspace(1, numTimeSteps, min(5, numTimeSteps)));
for i = 1:length(frameIdx)
    idx = frameIdx(i);
    if ~isempty(particlePositions{idx})
        % Plot a subset of particles to avoid overcrowding
        particlesToShow = min(100, numParticles);
        sampleIndices = round(linspace(1, numParticles, particlesToShow));
        
        positions = [particlePositions{idx}(sampleIndices).position];
        positions = reshape(positions, 3, [])';
        
        plot3(positions(:,1), positions(:,2), positions(:,3), '.', 'Color', [0.5, 0.5, 0.5, 0.3], 'MarkerSize', 3);
    end
end

% Add labels and legend
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Drone 3D UWB TDOA Localization with Particle Filter');
grid on;
legend('Anchors', 'Actual Trajectory', 'Estimated Trajectory', 'Particles');
axis equal;
view(45, 30); % Set view angle

% Plot localization error over time
subplot(2, 2, 2);
plot(timeVector, localizationErrors, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Error (m)');
title('Localization Error vs Time');
grid on;

% Plot RSSI measurements
subplot(2, 2, 4);
for i = 1:numAnchors
    plot(timeVector, smoothedRSSI(:,i), 'LineWidth', 1.2);
    hold on;
end
xlabel('Time (s)');
ylabel('RSSI (dBm)');
title('Smoothed RSSI Measurements');
grid on;
legend('Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4', 'Anchor 5', 'Location', 'best');

%% Helper Functions

% Initialize Kalman Filters for RSSI smoothing
function kfArray = initializeRSSIKalmanFilters(numAnchors)
    kfArray = cell(numAnchors, 1);
    for i = 1:numAnchors
        kf = struct();
        kf.state = -70;  % Initial RSSI estimate (dBm)
        kf.P = 5;        % Initial error covariance
        kf.Q = 0.1;      % Process noise covariance
        kf.R = 2;        % Measurement noise covariance
        kfArray{i} = kf;
    end
end

% Update Kalman Filter for RSSI smoothing
function kf = updateRSSIKalmanFilter(kf, measurement)
    % Prediction step
    % State remains the same (static model)
    % Predict error covariance
    kf.P = kf.P + kf.Q;
    
    % Update step
    % Calculate Kalman gain
    K = kf.P / (kf.P + kf.R);
    
    % Update state with measurement
    kf.state = kf.state + K * (measurement - kf.state);
    
    % Update error covariance
    kf.P = (1 - K) * kf.P;
end

% Convert RSSI to distance using path loss model
function distances = rssiToDistance(rssiValues, rssiAtOneMeter, pathLossExponent)
    % Path loss model: RSSI = RSSI_1m - 10*n*log10(d)
    % Solved for d: d = 10^((RSSI_1m - RSSI)/(10*n))
    distances = 10.^((rssiAtOneMeter - rssiValues) / (10 * pathLossExponent));
end

% Initialize particles
function particles = initializeParticles(numParticles, initialPosition, uncertainty)
    particles = struct('position', {}, 'weight', {});
    
    for i = 1:numParticles
        % Initialize particle positions around initial position with some uncertainty
        particles(i).position = initialPosition + uncertainty * randn(1, 3);
        particles(i).weight = 1/numParticles;  % Equal initial weights
    end
end

% Update particle filter with TDOA measurements
function particles = updateParticleFilter(particles, tdoaMeasurements, rssiValues, anchors, numParticles, effectiveThreshold)
    % 1. Predict step - add some random movement to particles (process noise)
    for i = 1:numParticles
        particles(i).position = particles(i).position + 0.05 * randn(1, 3);
    end
    
    % 2. Update step - calculate weights based on how well particles match TDOA measurements
    numAnchors = length(rssiValues);
    referenceAnchor = 1;  % First anchor is reference for TDOA
    
    for i = 1:numParticles
        % Calculate theoretical TDOA for this particle
        particleDistances = zeros(numAnchors, 1);
        
        for j = 1:numAnchors
            particleDistances(j) = norm(particles(i).position - anchors(j,:));
        end
        
        % Calculate particle's theoretical TDOA values
        particleTDOA = zeros(numAnchors-1, 1);
        for j = 2:numAnchors
            particleTDOA(j-1) = particleDistances(j) - particleDistances(referenceAnchor);
        end
        
        % Calculate weight based on TDOA error
        tdoaError = tdoaMeasurements - particleTDOA;
        tdoaLikelihood = exp(-0.5 * sum(tdoaError.^2) / 0.1);  % Gaussian likelihood
        
        % Also incorporate RSSI information
        rssiDistances = rssiToDistance(rssiValues, -65, 2.0);  % Estimated distances from RSSI
        rssiError = 0;
        
        for j = 1:numAnchors
            rssiError = rssiError + (particleDistances(j) - rssiDistances(j))^2;
        end
        
        rssiLikelihood = exp(-0.5 * rssiError / 1.0);  % Gaussian likelihood
        
        % Combine TDOA and RSSI likelihoods
        particles(i).weight = particles(i).weight * (0.7 * tdoaLikelihood + 0.3 * rssiLikelihood);
    end
    
    % 3. Normalize weights
    totalWeight = sum([particles.weight]);
    if totalWeight > 0
        for i = 1:numParticles
            particles(i).weight = particles(i).weight / totalWeight;
        end
    else
        % If all weights near zero, reset to equal weights
        for i = 1:numParticles
            particles(i).weight = 1/numParticles;
        end
    end
    
    % 4. Calculate effective number of particles
    weights = [particles.weight];
    nEffective = 1 / sum(weights.^2);
    
    % 5. Resample if needed
    if nEffective < effectiveThreshold
        % Systematic resampling
        particles = systematicResampling(particles, numParticles);
    end
end

% Systematic resampling algorithm for particle filter
function newParticles = systematicResampling(particles, numParticles)
    % Get cumulative sum of weights
    weights = [particles.weight];
    cumulativeSum = cumsum(weights);
    
    % Start at a random position
    startPos = rand(1) / numParticles;
    
    % Points to sample at
    samplePoints = startPos + (0:numParticles-1) / numParticles;
    
    % Create new particles array
    newParticles = struct('position', {}, 'weight', {});
    
    % Index for current original particle
    currentIndex = 1;
    
    % Resample
    for i = 1:numParticles
        % Find the particle corresponding to this sample p oint
        while samplePoints(i) > cumulativeSum(currentIndex) && currentIndex < numParticles
            currentIndex = currentIndex + 1;
        end
        
        % Copy the selected particle
        newParticles(i).position = particles(currentIndex).position;
        newParticles(i).weight = 1/numParticles;  % Reset weights to equal
    end
end

% Estimate position from particles
function estimatedPosition = estimatePositionFromParticles(particles)
    % Weighted average of particle positions
    weights = [particles.weight];
    positions = reshape([particles.position], 3, [])';
    
    % Ensure weights are properly sized for element-wise multiplication
    % weights should be a column vector for proper broadcasting
    weights = weights(:);
    
    % Use weighted sum with proper broadcasting
    estimatedPosition = sum(weights .* positions, 1) / sum(weights);
end