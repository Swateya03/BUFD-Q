% Load user locations from the provided .mat file
load('userLocations.mat'); % Ensure the 'userLocations' variable exists

% Simulation parameters
simulation_time = 30; % in minutes
time_step = 1; % 1-minute interval
num_users = size(userLocations, 1); % Number of users from loaded data

% Initialize a structure to store user requests
user_requests = struct();

% Simulate user requests with bounds
for t = 1:time_step:simulation_time
    % Ensure number of requests is between 220 and the total number of users
    min_requests = min(220, num_users); % Minimum bound adjusted
    max_requests = min(300, num_users); % Maximum bound adjusted
    num_requests = randi([min_requests, max_requests]); 
    
    % Randomly select users making requests
    requesting_users = randperm(num_users, num_requests);
    
    % Record the requests
    user_requests(t).time = t; % Current simulation time (minute)
    user_requests(t).requests = requesting_users; % IDs of requesting users
end

% Save the simulated user requests to a .mat file
save('userRequests.mat', 'user_requests');
disp('User requests simulation saved to userRequests.mat');

% Visualization 1: Bar chart of user requests over time
time_steps = arrayfun(@(x) x.time, user_requests);
num_requests = arrayfun(@(x) numel(x.requests), user_requests);

figure;
bar(time_steps, num_requests, 'FaceColor', [0.2, 0.6, 0.8]);
xlabel('Time (minutes)');
ylabel('Number of User Requests');
title('Number of User Requests Over Time');
grid on;

% Visualization 2: Heatmap of user requests
% Create a matrix where rows are users and columns are time steps
num_users = max(cellfun(@max, {user_requests.requests})); % Max user ID
heatmap_matrix = zeros(num_users, max(time_steps));

for t = 1:length(user_requests)
    heatmap_matrix(user_requests(t).requests, user_requests(t).time) = 1;
end

figure;
imagesc(heatmap_matrix);
colormap(hot);
colorbar;
xlabel('Time (minutes)');
ylabel('User ID');
title('Heatmap of User Requests Over Time');
set(gca, 'YDir', 'normal'); % Ensure user IDs are in ascending order


