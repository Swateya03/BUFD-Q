% Parameters
totalUsers = 300; % Total number of users
areaSize = 500; % Area dimensions (500m x 500m)
spreadPercent = 0.10; % Percentage of users to spread across the area
clusterPercentages = [0.30, 0.25, 0.20, 0.15]; % Percentages of users in each cluster

% Number of users in clusters and spread
usersSpread = round(totalUsers * spreadPercent);
usersInClusters = totalUsers - usersSpread;

% Calculate users per cluster based on percentages
numClusters = length(clusterPercentages);
usersPerCluster = round(usersInClusters * clusterPercentages);

% Cluster definitions
clusterCenters = [125, 125; 375, 125; 125, 375; 375, 375]; % Cluster centers [x, y]
clusterRadii = [120, 150, 100, 130]; % Larger radii for each cluster

% Initialize user coordinates
userLocations = []; % Matrix to store user locations

% Generate users for each cluster
for i = 1:numClusters
    centerX = clusterCenters(i, 1);
    centerY = clusterCenters(i, 2);
    radius = clusterRadii(i);
    numUsers = usersPerCluster(i);

    % Generate random users within the cluster radius
    theta = 2 * pi * rand(numUsers, 1); % Random angles
    r = radius * sqrt(rand(numUsers, 1)); % Random distances within radius
    x = centerX + r .* cos(theta); % X-coordinates
    y = centerY + r .* sin(theta); % Y-coordinates

    % Append user locations to the matrix
    userLocations = [userLocations; x, y];
end

% Generate spread-out users across the area
spreadX = areaSize * rand(usersSpread, 1); % Random X-coordinates
spreadY = areaSize * rand(usersSpread, 1); % Random Y-coordinates

% Append spread-out users to the matrix
userLocations = [userLocations; spreadX, spreadY];

% Save userLocations matrix to a MAT file for later use
matFileName = 'userLocations.mat';
save(matFileName, 'userLocations');

% Plot the user distribution
figure;
scatter(userLocations(:, 1), userLocations(:, 2), 20, 'b', 'filled'); % Users
hold on;

% Plot cluster boundaries and centers
for i = 1:numClusters
    viscircles(clusterCenters(i, :), clusterRadii(i), 'LineStyle', '--'); % Cluster boundaries
end
scatter(clusterCenters(:, 1), clusterCenters(:, 2), 100, 'r', 'filled'); % Cluster centers

% Labels and formatting
title('Realistic User Distribution with Saved Locations');
xlabel('X (meters)');
ylabel('Y (meters)');
xlim([0, areaSize]);
ylim([0, areaSize]);
grid on;
legend('Users', 'Cluster Boundaries', 'Cluster Centers');

% Display message about saved file
disp(['User locations have been saved to ', matFileName]);


