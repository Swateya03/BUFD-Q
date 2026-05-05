classdef MultiUAVEnv < rl.env.MATLABEnvironment
    properties
        % Environment parameters
        AreaSize = 500; % 500x500 m area
        GBSPosition = [250, 250, 0]; % Ground Base Station at the center
        MaxEnergy = 72; % Maximum energy (in Joules or Watt-hours)
        CoverageRadius = 136; % Initial coverage radius in meters
        EnergyPerMeter = 0.2; % Energy consumed per meter of movement
        
        % UAV properties
        NumUAVs = 4; % Number of UAVs
        UAVPositions; % Nx3 matrix for each UAV's [x, y, z] position
        UAVEnergies; % Array for each UAV's energy level
        
        % User properties
        NumUsers = 30;
        UserPositions;
        UserRequests;
        
        
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
        
        % Reward tracking
        PreviousCoveredUsers = 0;
    end
    
    properties (Access = protected)
        CurrentStep = 0; % Keeps track of steps in the episode
    end
    
    methods
        function this = MultiUAVEnv()
    % Define observation and action spaces explicitly without using properties
    ObservationInfo = rlNumericSpec([1, 12]); % 12 elements for 3 UAVs (4 for each UAV: [x, y, z, battery])
    ActionInfo = rlFiniteSetSpec([1, 2, 3, 4, 5, 6]); % Actions for 6 directions
    
    % Call superclass constructor with observation and action specs
    this@rl.env.MATLABEnvironment(ObservationInfo, ActionInfo);
    
    % Initialize parameters
    this.UAVEnergies = repmat(this.MaxEnergy, this.NumUAVs, 1);
    
    % Initialize UAV positions in a circular arrangement around the GBS
    radius = 150; % Distance from GBS for initial placement of UAVs (can adjust as needed)
    angleIncrement = 360 / this.NumUAVs; % Equal angle division for each UAV
    this.UAVPositions = zeros(this.NumUAVs, 3);
    
    for uavIdx = 1:this.NumUAVs
        % Calculate the angle for each UAV in radians
        angle = deg2rad((uavIdx - 1) * angleIncrement);
        
        % Set the x, y, and z positions for each UAV
        this.UAVPositions(uavIdx, :) = [
            this.GBSPosition(1) + radius * cos(angle), ... % x-coordinate
            this.GBSPosition(2) + radius * sin(angle), ... % y-coordinate
            50                                            % Initial altitude of 50 meters
        ];
    end
    
    % Randomize initial user positions and requests
    this.UserPositions = [rand(this.NumUsers, 1) * this.AreaSize, ...
                          rand(this.NumUsers, 1) * this.AreaSize, ...
                          zeros(this.NumUsers, 1)]; % Add z-coordinate for users
    this.UserRequests = ones(this.NumUsers, 1); % Initial requests for all users
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
        
        function [observation, reward, isDone, loggedSignals] = step(this, action)
    % Apply the same action to all UAVs and update each UAV's state
    totalCoverage = 0;
    uniqueUsersCovered = false(this.NumUsers, 1); % Track unique coverage

    for uavIdx = 1:this.NumUAVs
        initialPosition = this.UAVPositions(uavIdx, :);

        % Update UAV position based on the same action for all UAVs
        switch action
            case 1, this.UAVPositions(uavIdx, 3) = min(100, this.UAVPositions(uavIdx, 3) + 1); % Move up
            case 2, this.UAVPositions(uavIdx, 3) = max(15, this.UAVPositions(uavIdx, 3) - 1); % Move down
            case 3, this.UAVPositions(uavIdx, 1) = min(this.AreaSize, this.UAVPositions(uavIdx, 1) + 1); % East
            case 4, this.UAVPositions(uavIdx, 1) = max(0, this.UAVPositions(uavIdx, 1) - 1); % West
            case 5, this.UAVPositions(uavIdx, 2) = min(this.AreaSize, this.UAVPositions(uavIdx, 2) + 1); % North
            case 6, this.UAVPositions(uavIdx, 2) = max(0, this.UAVPositions(uavIdx, 2) - 1); % South
        end

        % Calculate energy consumed for movement
        distanceTraveled = norm(this.UAVPositions(uavIdx, :) - initialPosition);
        energyConsumed = this.EnergyPerMeter * distanceTraveled;
        this.UAVEnergies(uavIdx) = this.UAVEnergies(uavIdx) - energyConsumed;

        % Calculate coverage for each UAV
        distances = vecnorm(this.UserPositions - this.UAVPositions(uavIdx, :), 2, 2);
        for i = 1:this.NumUsers
            if distances(i) <= this.CoverageRadius
                pathLoss = this.calculatePathLoss(distances(i), this.UAVPositions(uavIdx, 3));
                if pathLoss <= this.L_threshold
                    uniqueUsersCovered(i) = true; % Mark user as covered by this UAV
                end
            end
        end
    end

    % Calculate total unique coverage
    totalCoverage = sum(uniqueUsersCovered);

    % Reward based on unique coverage and energy efficiency
    reward = totalCoverage - sum(this.UAVEnergies < 0) * 0.1; % Penalize for low energy

    % Update observation and check if done
    observation = reshape([this.UAVPositions, this.UAVEnergies], 1, []);
    isDone = all(this.UAVEnergies <= 0) || this.CurrentStep >= 100;
    this.CurrentStep = this.CurrentStep + 1;
    loggedSignals = [];

    % Plot environment
    this.plotEnvironment();
end

        
        function initialObs = reset(this)
    % Reset UAV energy levels and user requests
    this.UAVEnergies = repmat(this.MaxEnergy, this.NumUAVs, 1);
    this.CurrentStep = 0;
    this.UserRequests = ones(this.NumUsers, 1); % Reset all user requests
    
    % Reinitialize UAV positions in a circular arrangement around the GBS
    radius = 150; % Distance from GBS for initial placement of UAVs
    angleIncrement = 360 / this.NumUAVs; % Equal angle division for each UAV
    for uavIdx = 1:this.NumUAVs
        % Calculate the angle for each UAV in radians
        angle = deg2rad((uavIdx - 1) * angleIncrement);
        
        % Set the x, y, and z positions for each UAV
        this.UAVPositions(uavIdx, :) = [
            this.GBSPosition(1) + radius * cos(angle), ... % x-coordinate
            this.GBSPosition(2) + radius * sin(angle), ... % y-coordinate
            50                                            % Initial altitude of 50 meters
        ];
    end
    
    % Update observation
    initialObs = reshape([this.UAVPositions, this.UAVEnergies], 1, []);
    
    % Plot the environment at reset
    this.plotEnvironment();
end

        
        function plotEnvironment(this)
    % Clear the current figure
    clf;
    hold on;
    
    % Plot UAV positions as blue points with z-coordinates
    for uavIdx = 1:this.NumUAVs
        plot3(this.UAVPositions(uavIdx, 1), this.UAVPositions(uavIdx, 2), this.UAVPositions(uavIdx, 3), ...
              'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', sprintf('UAV %d', uavIdx));
    end
    
    % Plot user positions in 3D with z-coordinates
    plot3(this.UserPositions(:,1), this.UserPositions(:,2), this.UserPositions(:,3), 'ro', 'DisplayName', 'Users');
    
    % Plot coverage radius as a 3D sphere centered around each UAV
    [x, y, z] = sphere(20); % Create a sphere with 20 segments
    for uavIdx = 1:this.NumUAVs
        x_uav = x * this.CoverageRadius + this.UAVPositions(uavIdx, 1);
        y_uav = y * this.CoverageRadius + this.UAVPositions(uavIdx, 2);
        z_uav = z * this.CoverageRadius + this.UAVPositions(uavIdx, 3);
        surf(x_uav, y_uav, z_uav, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', sprintf('Coverage UAV %d', uavIdx)); % Set transparency for visibility
    end
    
    % Mark covered users in green if distances are available
    distances = vecnorm(this.UserPositions - this.GBSPosition, 2, 2);
    coveredUsers = distances <= this.CoverageRadius;
    plot3(this.UserPositions(coveredUsers,1), this.UserPositions(coveredUsers,2), this.UserPositions(coveredUsers,3), ...
          'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Covered Users');
    
    % Set plot limits and labels
    xlim([0 this.AreaSize]);
    ylim([0 this.AreaSize]);
    zlim([0 this.AreaSize]);
    title(sprintf('UAV Positions and Covered Users (Step %d)', this.CurrentStep));
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('Z Position (m)');
    view(3); % Set the view to 3D
    grid on;
    legend show; % Display legend to differentiate elements
    hold off;
    drawnow;
end

    end
end
