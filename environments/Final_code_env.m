classdef Final_code_env < rl.env.MATLABEnvironment
    % MultiUAVEnv Custom environment for multi-UAV deployment
    
    %% Properties
    properties
        % Define state and action space
        abscovereduser
        CurrentState
        UserRequests
        UAVPositions
        GBSPosition
        % Path loss parameters for suburban area
        Frequency = 2e9; % Frequency in Hz (2 GHz)
        Alpha = 0.2; % Specific attenuation due to rain
        Beta = 0.1; % Ducting coefficient
        Pnot = 0.01; % Empirical offset for ducting effects
        hr = 1.5; % Height of receiving antenna (User height in meters)
        Sigma = 4; % Standard deviation for shadowing in dB
        u = 0.5; % Ratio of built-up land area to total area
        v = 0.5; % Mean number of buildings per unit area
        w = 10; % Scale parameter for building height distribution
        L_threshold = 125; % Path loss threshold in dB
        
        UAVTrajectories  % Cell array to store the trajectory of each UAV
        MaxBattery = 100; % Maximum battery capacity in Joules
        UAVMass = 1;      % UAV mass in kg1
        Gravity = 9.81;   % Gravitational constant in m/s^2
        MaxDistanceToGBS
        expandedTable
        UserLocations
        ActiveUserLocations % Locations of active users at the current time step
        % Deployment properties
        CoveredUsers        % Number of users covered by each UAV
        OverlappedUsers     % Number of overlapped users for each UAV
        Velocities
        DistancesToGBS
        PreviousCoveredUsers  % Covered active users in the previous time step
        TotalCoveredUsersPerTimestamp   % Array to track total covered users at each timestamp
        MaxSteps
        CurrentStep
        CurrentTimeStep
        EnergyUsed
        TotalUniqueOverlappedUsers
    end
    
    %% Parameters
    properties
        NumUAVs
        AreaSize
        BatteryLevels
        RewardWeights  % Weights for reward components
    end
    properties
    CoveredLocIdx  % Logical array marking covered user locations for each UAV
    OverlappedLocIdx  % Logical array marking overlapped user locations
    end

   
    
    %% Public Methods
    methods
        % Constructor
        function this = Final_code_env()
            % Constructor logic
            numUAVs = 4;
            areaSize = [500, 500];  % Deployment area dimensions

            % Define state space
            stateSize = numUAVs * 7;  % Each UAV has 7 state variables
            observationInfo = rlNumericSpec([stateSize, 1], ...
                'LowerLimit', -inf, 'UpperLimit', inf, ...
                'Name', 'UAVState');

            azimuthValues = [-15, -12, -10,0, 10,12, 15];
            elevationValues = [-10, -7, -5, -3, 0, 3, 5, 7, 10];
            radialValues = [1, 1.4, 1.8, 2.2, 2.4];

            numAzimuth = length(azimuthValues);
            numElevation = length(elevationValues);
            numRadial = length(radialValues);
            numActions = numAzimuth * numElevation * numRadial;  % Total number of discrete actions

            % Initialize action space
            actionInfo = rlFiniteSetSpec(1:numActions);  % 315 discrete actions
            actionInfo.Name = 'UAVActions';

            


            % Call superclass constructor
            this = this@rl.env.MATLABEnvironment(observationInfo, actionInfo);

            % Initialize parameters
            this.NumUAVs = numUAVs;
            this.AreaSize = areaSize;
            this.BatteryLevels = 100 * ones(1, numUAVs);
            this.RewardWeights = [0.25, 0.26, 1, 0.01];
            this.MaxDistanceToGBS = 150;
            this.MaxSteps = 30;
            this.reset();
            this.GBSPosition = [this.AreaSize(1) / 2, this.AreaSize(2) / 2, 0];  % GBS at the center
            this.EnergyUsed = zeros(1, this.NumUAVs);
            % Update property
            this.ActionInfo = actionInfo; % Ensure this is accessible

        end

        % Reset Method
function InitialObservation = reset(this)
    % Reset environment to initial state at the beginning of an episode
    
    %% Initialize CurrentTimeStep
    this.CurrentTimeStep = 1;  % Start at the first time step
    this.TotalCoveredUsersPerTimestamp = zeros(this.MaxSteps, 1);  % Preallocate for all timestamps

    %% 1. Initialize UAV Positions
    radius = 125;  % Deployment radius (meters)
    gbsPosition = [this.AreaSize(1) / 2, this.AreaSize(2) / 2, 0];  % GBS at center
    angles = linspace(0, 360, this.NumUAVs + 1);  % Angles for UAVs
    angles = angles(1:end-1);  % Exclude duplicate

    this.UAVPositions = zeros(this.NumUAVs, 3);  % [x, y, z]
    for i = 1:this.NumUAVs
        this.UAVPositions(i, :) = [...
            gbsPosition(1) + radius * cosd(angles(i)), ...
            gbsPosition(2) + radius * sind(angles(i)), ...
            40];
    end
    this.UAVPositions = this.UAVPositions;
    this.UAVTrajectories = cell(this.NumUAVs, 1);
    for i = 1:this.NumUAVs
        this.UAVTrajectories{i} = this.UAVPositions(i, :);  % Start with initial position
    end


    %% 2. Initialize UAV Battery Levels
    this.BatteryLevels = 100 * ones(1, this.NumUAVs);

    %% 3. Load User Data
    userData = load('correctedProcessedUserData.mat');
    this.expandedTable = userData.processedTable;
    this.UserLocations = [this.expandedTable.user_id_x, this.expandedTable.user_id_y];
    this.UserRequests = table2array(this.expandedTable(:, 4:end));
    

    %% 4. Reset Metrics
    this.CoveredUsers = zeros(1, this.NumUAVs);
    this.OverlappedUsers = zeros(1, this.NumUAVs);
    this.PreviousCoveredUsers = zeros(1, this.NumUAVs);  % Initialize to zero for all UAVs


    %% 5. Initialize UAV Velocities
    this.Velocities = 2 * ones(1, this.NumUAVs);

    %% 6. Calculate Distance to GBS
    this.DistancesToGBS = radius * ones(1, this.NumUAVs);

    %% 7. Construct Initial State
    InitialObservation = zeros(this.NumUAVs * 7, 1);  % Preallocate state vector
    for i = 1:this.NumUAVs
        % Construct the UAV state vector (7 elements)
        stateVector = [...
            this.UAVPositions(i, :), ...  % x, y, z (3 elements)
            this.BatteryLevels(i), ...    % Battery level (1 element)
            this.CoveredUsers(i), ...     % Covered users (1 element)
            this.DistancesToGBS(i), ...   % Distance to GBS (1 element)
            this.Velocities(i)];          % Velocity (1 element)
        
        % Verify the state vector is of size [1, 7]
        disp(['UAV ', num2str(i), ' State Vector: ', mat2str(stateVector)]);

        % Assign the state vector to InitialObservation
        InitialObservation((i-1)*7 + 1:i*7) = stateVector(:);
    end
end
function [NextObservation, Reward, IsDone, LoggedSignals] = step(this, Action)
    % STEP Simulates the environment for one step given an action.
    % Action is the index of the selected action (1 to 315 per UAV)
    
    %% Update Previous Covered Users
    this.PreviousCoveredUsers = this.CoveredUsers;
    
    %% 1. Update Active Users
    % Determine active users at the current time step
    currentTime = this.CurrentTimeStep;
    activeUsersIdx = find(this.UserRequests(:, currentTime) == 1);  % Active users at time t
    this.ActiveUserLocations = this.UserLocations(activeUsersIdx, :);  % Filter locations

    numActiveUsers = length(activeUsersIdx);
    % Debugging: Log the active user indices and count
    disp(['Timestamp: ', num2str(currentTime)]);
    disp(['Number of Active Users: ', num2str(numActiveUsers)]);
    

    % Initialize Covered and Overlapped User Locations
    this.CoveredLocIdx = false(size(this.ActiveUserLocations, 1), this.NumUAVs);  % Covered users for each UAV
    this.OverlappedLocIdx = false(size(this.ActiveUserLocations, 1), 1);  % Overlapped users overall
    %% 2. Action Interpretation
    % Convert discrete action index to azimuth, elevation, and radial distance
    azimuthValues = [-15, -12, -10,0, 10,12, 15];
    elevationValues = [-10, -7, -5, -3, 0, 3, 5, 7, 10];
    radialValues = [1, 1.4, 1.8, 2.2, 2.4];
    

    [azimuthIndex, elevationIndex, radialIndex] = ind2sub([7, 9, 5], Action);
    deltaAzimuth = azimuthValues(azimuthIndex);
    deltaElevation = elevationValues(elevationIndex);
    deltaRadial = radialValues(radialIndex);
    % Display the selected action in terms of spherical components
    disp(['Selected Action: \Delta\phi = ', num2str(deltaElevation), ...
          ', \Delta\theta = ', num2str(deltaAzimuth), ...
          ', \Delta r = ', num2str(deltaRadial)]);
    %% 3. Update UAV Positions
    for i = 1:this.NumUAVs
        % Convert spherical to Cartesian updates
        deltaX = deltaRadial .* cosd(deltaAzimuth) .* cosd(deltaElevation);
        deltaY = deltaRadial .* sind(deltaAzimuth) .* cosd(deltaElevation);
        deltaZ = deltaRadial .* sind(deltaElevation);

        % Default to no vertical movement if deltaZ is empty
        if isempty(deltaZ)
            deltaZ = 0;
        end

        % Check and set default UAV height if uninitialized
        if isempty(this.UAVPositions(i, 3))
            this.UAVPositions(i, 3) = 40; % Default starting height
        end

        % Calculate new height and enforce constraints
        newHeight = this.UAVPositions(i, 3) + deltaZ;
        newHeight = max(20, min(100, newHeight)); % Ensure height is within bounds

        % Update UAV position
        this.UAVPositions(i, :) = this.UAVPositions(i, :) + [deltaX, deltaY, 0]; % Update x and y only
        this.UAVPositions(i, 3) = newHeight; % Update z separately after enforcing constraints

        % Enforce boundary conditions (within the deployment area)
        this.UAVPositions(i, 1) = max(0, min(this.AreaSize(1), this.UAVPositions(i, 1)));
        this.UAVPositions(i, 2) = max(0, min(this.AreaSize(2), this.UAVPositions(i, 2)));

        % Log updated positions for debugging
        disp(['UAV ', num2str(i), ' Updated Position: ', mat2str(this.UAVPositions(i, :))]);
     end


    for i = 1:this.NumUAVs
        % Append the current position to the trajectory
        this.UAVTrajectories{i} = [this.UAVTrajectories{i}; this.UAVPositions(i, :)];
    end


   %% 4. Update Coverage and Overlap Metrics
% Initialize total unique overlapped users
this.TotalUniqueOverlappedUsers = 0;

% Loop through UAVs
for i = 1:this.NumUAVs
    coveredUsers = 0;  % Reset covered users counter for each UAV
    overlappedUsers = 0;  % Reset overlapped users counter for each UAV
    
    for j = 1:size(this.ActiveUserLocations, 1)
        distance = norm(this.UAVPositions(i, :) - [this.ActiveUserLocations(j, :), 0]);
        pathLoss = this.calculatePathLoss(distance, this.UAVPositions(i, 3));
        isCovered = false;  % Flag to track if the user is already covered

        % Compare path loss with other UAVs to determine primary coverage
        if pathLoss <= this.L_threshold
            primaryUAV = i;  % Assume current UAV is primary for this user
            minPathLoss = pathLoss;

            % Check if any other UAV has lower path loss
            for k = 1:this.NumUAVs
                if k ~= i
                    distanceOtherUAV = norm(this.UAVPositions(k, :) - [this.ActiveUserLocations(j, :), 0]);
                    pathLossOtherUAV = this.calculatePathLoss(distanceOtherUAV, this.UAVPositions(k, 3));

                    if pathLossOtherUAV < minPathLoss
                        minPathLoss = pathLossOtherUAV;
                        primaryUAV = k;  % Update primary UAV for this user
                    end
                end
            end

            % Only count the user for the UAV with the lowest path loss
            if primaryUAV == i
                this.CoveredLocIdx(j, i) = true;
                coveredUsers = coveredUsers + 1;
                isCovered = true;
            end
        end

        % Check for overlaps even if the user is not primarily covered by this UAV
        if pathLoss <= this.L_threshold && isCovered
            overlapCount = 0;

            % Check for overlaps with other UAVs
            for k = 1:this.NumUAVs
                if k ~= i
                    distanceOtherUAV = norm(this.UAVPositions(k, :) - [this.ActiveUserLocations(j, :), 0]);
                    pathLossOtherUAV = this.calculatePathLoss(distanceOtherUAV, this.UAVPositions(k, 3));

                    if pathLossOtherUAV <= this.L_threshold
                        overlapCount = overlapCount + 1;
                        break;
                    end
                end
            end

            % If the user is covered by more than one UAV, mark it as overlapped
            if overlapCount > 0 && ~this.OverlappedLocIdx(j) % Check if already counted
                this.OverlappedLocIdx(j) = true;  % Mark user as overlapped
                overlappedUsers = overlappedUsers + 1;
                this.TotalUniqueOverlappedUsers = this.TotalUniqueOverlappedUsers + 1; % Unique count
            end
        end
    end

    this.CoveredUsers(i) = coveredUsers;
    this.OverlappedUsers(i) = overlappedUsers;  % Per-UAV overlapped users
end



    %% 5. Update Battery Levels
    for i = 1:this.NumUAVs
        % Calculate energy usage based on movement and phase
        altitudeChange = abs(deltaZ);
        if deltaZ > 0
            % Energy used when increasing altitude
            energyUsed = 1.1 * 1 + this.UAVMass * this.Gravity * altitudeChange + ...
                         0.8 * 1 + this.UAVMass * this.Velocities(i) * 1;
        elseif deltaZ < 0
            % Energy used when decreasing altitude
            energyUsed = this.UAVMass * this.Gravity * altitudeChange + ...
                         0.8 * 1 + this.UAVMass * this.Velocities(i) * 1;
        else
            % Energy used during cruise
            energyUsed = 0.8 * 1 + this.UAVMass * this.Velocities(i) * 1;
        end

        % Update battery level as a percentage
        batteryUsagePercentage = (energyUsed / this.MaxBattery) * 100;
        this.BatteryLevels(i) = max(0, this.BatteryLevels(i) - batteryUsagePercentage);
        % Update energy used (if defined as a property)
        this.EnergyUsed(i) = energyUsed;
    end

    %% 6. Compute Reward
    %% 6. Compute Reward
Reward = 0;

for i = 1:this.NumUAVs
    % --- Coverage Reward ---
    this.abscovereduser(i) = this.CoveredUsers(i);
    activeUserRatio = this.abscovereduser(i) / sum(activeUsersIdx); % Avoid division by zero
    fprintf('covered users %d for uav %d',this.CoveredUsers(i),i);
    
    % Positive reward for improving coverage
    Reward = Reward + this.RewardWeights(1)*activeUserRatio;
    

    % --- Overlap Reward ---
    if this.OverlappedUsers(i) <= 5
        Reward = Reward + this.RewardWeights(2) * (5-this.OverlappedUsers(i));
    else
        Reward = Reward - this.RewardWeights(2) * (this.OverlappedUsers(i) - 5);
    end

    % --- Battery Penalty ---
    energyUsed = this.EnergyUsed(i); % Assume energyUsed is available per UAV
    if activeUserRatio <= 0.2
        batteryPenalty = this.RewardWeights(3) * energyUsed;
        Reward = Reward - batteryPenalty;
    end

    % --- Proximity to GBS Reward ---
    distanceToGBS = norm(this.UAVPositions(i, :) - this.GBSPosition);
    if distanceToGBS <= this.MaxDistanceToGBS
        proximityReward = this.RewardWeights(4) * (this.MaxDistanceToGBS-distanceToGBS );
        Reward = Reward + proximityReward;
    else
        proximityPenalty = this.RewardWeights(4) * (distanceToGBS-this.MaxDistanceToGBS);
        Reward = Reward - proximityPenalty;
    end
end



   
    %% 7. Episode Termination
    IsDone = this.CurrentTimeStep >= this.MaxSteps || all(this.BatteryLevels <= 0);
    


    %% 8. Update State and Return Values
    % Update state vector
    NextObservation = zeros(this.NumUAVs * 7, 1);
    for i = 1:this.NumUAVs
        stateVector = zeros(1, 7);  % Preallocate with fixed size
        stateVector(1:3) = this.UAVPositions(i, :);  % [x, y, z]
        stateVector(4) = this.BatteryLevels(i);      % Battery level
        stateVector(5) = this.CoveredUsers(i);       % Covered users
        stateVector(6) = this.DistancesToGBS(i);     % Distance to GBS
        stateVector(7) = this.Velocities(i);         % Velocity
        
        
        % Assign the state vector to NextObservation
        NextObservation((i-1)*7 + 1:i*7) = stateVector(:);
    end

    % Logged signals for debugging
    LoggedSignals = struct('BatteryLevels', this.BatteryLevels, ...
                           'CoveredUsers', this.CoveredUsers, ...
                           'OverlappedUsers', this.OverlappedUsers);

    
    
    % Calculate the total number of covered users across all UAVs
    totalCoveredUsers = sum(this.CoveredUsers);

    % Store the total covered users for the current timestamp
    this.TotalCoveredUsersPerTimestamp(this.CurrentTimeStep) = totalCoveredUsers;
    
    % Debugging (optional)
    disp(['Current Timestamp: ', num2str(this.CurrentTimeStep), ...
      ', Total Covered Users: ', num2str(totalCoveredUsers)]);
   % Increment time step only if the episode is not done
    if ~IsDone
        this.CurrentTimeStep = this.CurrentTimeStep + 1;
    end
    

end

function L = calculatePathLoss(this, distance, ht)
            % Calculate LOS Path Loss with dynamic ht
            FSPL = 20*log10(distance) + 20*log10(this.Frequency) + 20*log10(4*pi/3e8);
            L_rain = this.Alpha * distance;
            L_duct = this.Beta * distance + this.Pnot;
            L_LOS = FSPL + L_rain + L_duct;
            
            % Calculate NLOS Path Loss
            L_reflection = 10*log10(1 + (ht * this.hr / distance^2)^2);
            L_shadow = this.Sigma * randn(); % Shadowing effect with Gaussian variable
            L_NLOS = L_LOS + L_reflection + L_shadow;
            
            % Calculate p(LOS) using parameters a and b
            a = this.u * this.v;
            b = this.w;
            theta = atan(this.hr / distance); % Elevation angle
            p_LOS = 1 / (1 + a * exp(-b * (theta - a)));
            p_NLOS = 1 - p_LOS;
            
            % Calculate average path loss
            L = p_LOS * L_LOS + p_NLOS * L_NLOS;
        end
    end 
methods
    function visualizeEnvironment(this, episode)
    % VISUALIZEENVIRONMENT Visualizes the current state of the environment

    figure(1);
    clf;
    hold on;

    % Plot GBS position with a new symbol ('x')
    scatter3(this.GBSPosition(1), this.GBSPosition(2), this.GBSPosition(3), ...
             200, 'r', 'x', 'LineWidth', 2, 'DisplayName', 'GBS');  % Red 'x'

    % Plot UAV trajectories
    for uavIdx = 1:this.NumUAVs
        trajectory = this.UAVTrajectories{uavIdx};
        plot3(trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), ...
              'LineWidth', 1.5, 'DisplayName', ['Trajectory of UAV ', num2str(uavIdx)]);
    end

    % Plot UAV positions with different symbols
    uavSymbols = {'o', 's', 'd', '^', 'v', '*', 'p', 'h'};  % Symbols for up to 8 UAVs
    for uavIdx = 1:this.NumUAVs
        scatter3(this.UAVPositions(uavIdx, 1), this.UAVPositions(uavIdx, 2), this.UAVPositions(uavIdx, 3), ...
                 100, 'b', uavSymbols{uavIdx}, 'filled', ...
                 'DisplayName', ['UAV ', num2str(uavIdx)]);
    end

    % Plot covered users for each UAV with the same symbol but in orange
    for uavIdx = 1:this.NumUAVs
        % Find users covered by the specific UAV
        coveredIdx = this.CoveredLocIdx(:, uavIdx);  % Logical index for covered users by this UAV
        scatter3(this.ActiveUserLocations(coveredIdx, 1), ...
                 this.ActiveUserLocations(coveredIdx, 2), ...
                 zeros(sum(coveredIdx), 1), ...
                 50, [1, 0.5, 0], uavSymbols{uavIdx}, ...  % Orange color [1, 0.5, 0]
                 'DisplayName', ['Users Covered by UAV ', num2str(uavIdx)]);
    end

    % Plot overlapped users with bold red markers
    overlappedIdx = this.OverlappedLocIdx;  % Logical index for overlapped users
    scatter3(this.ActiveUserLocations(overlappedIdx, 1), ...
             this.ActiveUserLocations(overlappedIdx, 2), ...
             zeros(sum(overlappedIdx), 1), ...
             100, 'r', 'o', 'LineWidth', 2, ...
             'DisplayName', 'Overlapped Users');  % Bold red circle

    % Plot non-covered users
    nonCoveredIdx = ~any(this.CoveredLocIdx, 2);  % Users not covered by any UAV
    scatter3(this.ActiveUserLocations(nonCoveredIdx, 1), ...
             this.ActiveUserLocations(nonCoveredIdx, 2), ...
             zeros(sum(nonCoveredIdx), 1), ...
             50, 'k', 'd', 'DisplayName', 'Non-Covered Users');  % Black diamonds

    % Number of active users
    numActiveUsers = size(this.ActiveUserLocations, 1);

    % Display the number of active users in the plot title
    title(['UAV Positions and User Coverage - Episode ', num2str(episode), ...
           ', Timestamp ', num2str(this.CurrentTimeStep), ...
           ', Active Users: ', num2str(numActiveUsers)]);
       
    % Configure plot
    xlim([0, this.AreaSize(1)]);
    ylim([0, this.AreaSize(2)]);
    zlim([0, 120]);
    xlabel('X Position (meters)');
    ylabel('Y Position (meters)');
    zlabel('Z Position (meters)');
    legend show;
    grid on;
    view(3);
    hold off;
    drawnow;
end

end

end

