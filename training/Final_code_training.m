function Final_code_training(env)
    % Training configurations
    configurations = [
        0.07, 0.95;
        
    ];
    numConfigs = size(configurations, 1);
    numEpisodes = 100;
    maxSteps = 30;

    % Preallocate logs
    rewardsLog = zeros(numConfigs, numEpisodes); % Total reward per episode
    coverageLog = zeros(numConfigs, numEpisodes); % Total coverage per episode
    timestampsLog = zeros(numConfigs, numEpisodes); % Timestamps reached per episode
    overlappedLog = zeros(numConfigs, numEpisodes); % Total unique overlapped users per episode
    energyConsumedLog = zeros(numConfigs, numEpisodes); % Total energy consumed per episode

    % Loop over configurations
    for configIdx = 1:numConfigs
        learningRate = configurations(configIdx, 1);
        discountFactor = configurations(configIdx, 2);
        fprintf('\nStarting Training for Config %d: LR = %.2f, DF = %.2f\n', configIdx, learningRate, discountFactor);

        % Initialize Q-table
        ObservationInfo = getObservationInfo(env);
        stateSize = prod(ObservationInfo.Dimension);
        actionSize = 315;
        QTable = 0.01 * rand(stateSize, actionSize);

        for episode = 1:numEpisodes
            % Reset environment
            InitialObservation = env.reset();
            currentState = encodeState(InitialObservation, stateSize);
            totalReward = 0;
            totalCoverage = 0;
            totalEnergyConsumed = 0; % Initialize energy consumption for the episode

            % Temperature parameter for Boltzmann policy
            temperature = max(1 - episode / numEpisodes, 0.1); % Gradually decrease temperature

            for step = 1:maxSteps
                % Select action using Boltzmann policy
                action = boltzmannPolicy(QTable, currentState, temperature);

                % Step in the environment
                [NextObservation, Reward, IsDone, LoggedSignals] = env.step(action);
                nextState = encodeState(NextObservation, stateSize);

                % Update Q-table
                bestFutureValue = max(QTable(nextState, :));
                QTable(currentState, action) = QTable(currentState, action) + ...
                    learningRate * (Reward + discountFactor * bestFutureValue - QTable(currentState, action));

                % Accumulate metrics
                totalReward = totalReward + Reward;
                totalCoverage = totalCoverage + sum(env.CoveredUsers);
                totalEnergyConsumed = totalEnergyConsumed + sum(LoggedSignals.BatteryLevels); % Sum of battery usage

                % Update state
                currentState = nextState;

                % Break if done
                if IsDone
                    break;
                end
            end

            % Log metrics
            rewardsLog(configIdx, episode) = totalReward;
            coverageLog(configIdx, episode) = totalCoverage;
            timestampsLog(configIdx, episode) = env.CurrentTimeStep;
            overlappedLog(configIdx, episode) = env.TotalUniqueOverlappedUsers;
            energyConsumedLog(configIdx, episode) = totalEnergyConsumed;

            fprintf('Config %d, Episode %d: Reward = %.2f, Coverage = %d, Timestamps = %d, Energy Consumed = %.2f\n', ...
                configIdx, episode, totalReward, totalCoverage, env.CurrentTimeStep, totalEnergyConsumed);
        end
    end

    % Save Q-table for each configuration
    save('TrainedQTables.mat', 'QTable');
    fprintf('Training complete! Q-tables saved.\n');

    % Generate plots
    generatePlots(rewardsLog, coverageLog, timestampsLog, overlappedLog, energyConsumedLog, numEpisodes);
end

function generatePlots(rewardsLog, coverageLog, timestampsLog, overlappedLog, energyConsumedLog, numEpisodes)
    % Define line styles and markers
    lineStyles = {'-', '--', '-.', ':'};
    markers = {'o', 's', '^', 'd', '*', 'x', '+', 'p', 'h'};
    smoothingWindow = 50; % Define the window size for smoothing

    % Function to smooth the data using a moving average
    smoothData = @(data) movmean(data, smoothingWindow);

    % Plot rewards
    figure;
    hold on;
    smoothedRewards = smoothData(rewardsLog(1, :)); % Apply smoothing
    plot(1:numEpisodes, smoothedRewards, ...
        'LineStyle', lineStyles{1}, ...
        'Marker', markers{1}, ...
        'LineWidth', 1.5);
    xlabel('Episodes');
    ylabel('Reward');
    title('Reward vs. Episodes');
    grid on;
    hold off;

    % Plot coverage
    figure;
    hold on;
    smoothedCoverage = smoothData(coverageLog(1, :)); % Apply smoothing
    plot(1:numEpisodes, smoothedCoverage, ...
        'LineStyle', lineStyles{1}, ...
        'Marker', markers{1}, ...
        'LineWidth', 1.5);
    xlabel('Episodes');
    ylabel('Coverage');
    title('Coverage vs. Episodes');
    grid on;
    hold off;

    % Plot overlapped users
    figure;
    hold on;
    smoothedOverlapped = smoothData(overlappedLog(1, :)); % Apply smoothing
    plot(1:numEpisodes, smoothedOverlapped, ...
        'LineStyle', lineStyles{1}, ...
        'Marker', markers{1}, ...
        'LineWidth', 1.5);
    xlabel('Episodes');
    ylabel('Overlapped Users');
    title('Overlapped Users vs. Episodes');
    grid on;
    hold off;

    % Plot Episodes vs Energy Consumed
    figure;
    hold on;
    smoothedEnergyConsumed = smoothData(energyConsumedLog(1, :)); % Apply smoothing
    plot(1:numEpisodes, smoothedEnergyConsumed, ...
          'LineStyle', lineStyles{1}, ...
          'Marker', markers{1}, ...
          'LineWidth', 1.5);
    xlabel('Episodes');
    ylabel('Energy Consumption');
    title('Energy Consumed vs Episode');
    grid on;
    hold off;
end
function action = boltzmannPolicy(QTable, currentState, temperature)
    % Boltzmann policy for action selection
    qValues = QTable(currentState, :);
    expQ = exp(qValues / temperature); % Compute exponentials of Q-values
    probabilities = expQ / sum(expQ); % Normalize to create a probability distribution

    % Select an action based on the computed probabilities
    cumulativeProb = cumsum(probabilities);
    randomValue = rand;
    action = find(cumulativeProb >= randomValue, 1);
end



function stateIndex = encodeState(observation, maxStates)
    % Ensure observation is finite and scaled correctly
    scaledObservation = floor(abs(observation) / max(abs(observation(:))) * (maxStates - 1));
    stateIndex = sum(scaledObservation) + 1;
    stateIndex = min(max(stateIndex, 1), maxStates);  % Clamp within valid range
end


