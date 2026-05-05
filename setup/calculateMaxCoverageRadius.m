% Main script code
UAVHeight = 50; % Initial UAV height in meters
L_threshold = 125; % Path loss threshold for voice/text services in dB
maxRadius = calculatMaxCoverageRadius(UAVHeight, L_threshold);
fprintf('Maximum coverage radius for UAV: %.2f meters\n', maxRadius);

% Local function definition
function maxRadius = calculatMaxCoverageRadius(UAVHeight, L_threshold)
    % Parameters
    Frequency = 2e9; % Frequency in Hz (2 GHz)
    Alpha = 0.2; % Specific attenuation due to rain
    Beta = 0.1; % Ducting coefficient
    Pnot = 0.01; % Empirical offset for ducting effects
    hr = 1.5; % Height of receiving antenna (User height in meters)
    Sigma = 4; % Standard deviation for shadowing in dB
    u = 0.5; % Ratio of built-up land area to total area
    v = 0.5; % Mean number of buildings per unit area
    w = 10; % Scale parameter for building height distribution

    % Initialize distance parameters
    distance = 1; % Start at 1 meter
    stepSize = 1; % Step size for distance increase (in meters)
    maxRadius = 0; % Initialize max radius to 0

    % Iteratively calculate path loss until it exceeds threshold
    while true
        % Calculate Free-Space Path Loss (FSPL)
        FSPL = 20*log10(distance) + 20*log10(Frequency) + 20*log10(4 * pi / 3e8);
        
        % Calculate Rain Attenuation
        L_rain = Alpha * distance;
        
        % Calculate Ducting Effects
        L_duct = Beta * distance + Pnot;
        
        % LOS Path Loss (L_LOS)
        L_LOS = FSPL + L_rain + L_duct;

        % Calculate Reflection Loss for NLOS
        L_reflection = 10 * log10(1 + (UAVHeight * hr / distance^2)^2);

        % Calculate Shadowing Loss (average effect)
        L_shadow = Sigma; % Average shadowing effect, without random fluctuation

        % NLOS Path Loss (L_NLOS)
        L_NLOS = L_LOS + L_reflection + L_shadow;

        % Calculate p(LOS) based on distance
        a = u * v;
        b = w;
        theta = atan(hr / distance); % Elevation angle
        p_LOS = 1 / (1 + a * exp(-b * (theta - a)));
        p_NLOS = 1 - p_LOS;

        % Calculate Average Path Loss (L_avg)
        L_avg = p_LOS * L_LOS + p_NLOS * L_NLOS;

        % Check if path loss exceeds threshold
        if L_avg > L_threshold
            maxRadius = distance - stepSize; % Set max radius to last valid distance
            break;
        end

        % Increment distance
        distance = distance + stepSize;
    end
end

