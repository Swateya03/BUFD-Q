% COV_RANGE.m
% -------------------------------------------------------------------------
% This script calculates the coverage radius for different environments
% and plots Coverage Radius vs Altitude.

% Initialization
clear; clc;

% Define constants
f = 2.4e9; % Frequency in Hz (e.g., 2.4 GHz)
c = 3e8;   % Speed of light in m/s
altitudes = 50:10:500; % Altitude range (50m to 500m)

% Define environment parameters (example values)
environments = {'Suburban', 'Urban', 'DenseUrban', 'HighriseUrban'};
env_params.Suburban.a = 1; env_params.Suburban.b = 2;
env_params.Suburban.u = 1; env_params.Suburban.v = 3; env_params.Suburban.p = 0.9;

env_params.Urban.a = 2; env_params.Urban.b = 3;
env_params.Urban.u = 2; env_params.Urban.v = 4; env_params.Urban.p = 0.7;

env_params.DenseUrban.a = 3; env_params.DenseUrban.b = 4;
env_params.DenseUrban.u = 3; env_params.DenseUrban.v = 5; env_params.DenseUrban.p = 0.5;

env_params.HighriseUrban.a = 4; env_params.HighriseUrban.b = 5;
env_params.HighriseUrban.u = 4; env_params.HighriseUrban.v = 6; env_params.HighriseUrban.p = 0.3;

% Initialize storage for results
coverage_radius = struct();

% Main Loop
for env_name = environments
    env = env_name{1};
    params = env_params.(env);
    
    % Initialize radius array
    R = nan(size(altitudes));
    
    for i = 1:length(altitudes)
        h = altitudes(i); % Current altitude
        
        % Define coverage function
        coverage_function = @(r) compute_loss(r, h, f, c, params) - 100; % Threshold: 100 dB
        
        % Adjust bounds dynamically
        lower_bound = 1;
        upper_bound = 2000;
        f_lower = coverage_function(lower_bound);
        f_upper = coverage_function(upper_bound);
        
        % Find valid bounds if no sign change
        if f_lower * f_upper > 0
            warning('No sign change for altitude %.1f and environment %s', h, env);
            % Dynamically adjust bounds
            lower_bound = max(1, lower_bound * 0.1);
            upper_bound = min(2000, upper_bound * 1.5);
            f_lower = coverage_function(lower_bound);
            f_upper = coverage_function(upper_bound);
            
            if f_lower * f_upper > 0
                continue; % Skip if still no root
            end
        end
        
        % Try to find the root
        try
            R(i) = fzero(coverage_function, [lower_bound, upper_bound]);
        catch ME
            warning('Root not found for altitude %.1f and environment %s: %s', h, env, ME.message);
        end
    end
    
    % Store results
    coverage_radius.(env) = R;
end

% Plotting
figure;
hold on;
for env_name = environments
    env = env_name{1};
    plot(altitudes, coverage_radius.(env), 'DisplayName', env);
end
hold off;

xlabel('Altitude (m)');
ylabel('Coverage Radius (m)');
title('Coverage Radius vs Altitude for Different Environments');
legend show;
grid on;

% -------------------------------------------------------------------------
% Function Definitions
% -------------------------------------------------------------------------
function L = compute_loss(r, h, f, c, params)
    % Compute the path loss for Line-of-Sight (LoS) and Non-Line-of-Sight (NLoS)
    FSPL = @(d) 20 * log10(d) + 20 * log10(f) - 20 * log10(c); % Example FSPL formula
    
    % Calculate total loss
    L_LoS = FSPL(r) + params.a * h + params.b;
    L_NLoS = FSPL(r) + params.u * h + params.v + 10 * log10(1 + (h^2 / r.^2));
    L = params.p * L_LoS + (1 - params.p) * L_NLoS;
end






