% Parameters
deploymentArea = 500 * 500; % Total area (500m x 500m)
totalUsers = 300; % Total number of users
numUAVs = 4; % Number of UAVs to deploy

% Calculate coverage area and radius per UAV
areaPerUAV = deploymentArea / numUAVs; % Area covered by each UAV
radiusPerUAV = sqrt(areaPerUAV / pi); % Radius for each UAV

% Calculate users covered per UAV
userDensity = totalUsers / deploymentArea; % Users per square meter
usersPerUAV = areaPerUAV * userDensity;

% Display results
fprintf('Flood Situation UAV Deployment:\n');
fprintf('Deployment Area: %.2f m²\n', deploymentArea);
fprintf('Total Users: %d\n', totalUsers);
fprintf('Number of UAVs: %d\n', numUAVs);
fprintf('Coverage Area per UAV: %.2f m²\n', areaPerUAV);
fprintf('Coverage Radius per UAV: %.2f meters\n', radiusPerUAV);
fprintf('Users covered per UAV: %.2f users\n', usersPerUAV);

