% Komal Porwal
% 19th September 2024
% Problem 1.3
function [trim_state, trim_control,optimal_trim_var] = find_trim_condition(trim_def, aircraft_parameters)
    % Initial guess for trim variables: [alpha, elevator, throttle]
    trim_var0 = [5 * pi / 180; 0; 0.5];  % Example: [5 deg, 0, 50% throttle]
   % trim_def = struct('airspeed', 18, 'altitude', 1800); 
    %aircraft_parameters.m = 5.74;  % Aircraft mass in kg
    %aircraft_parameters.S = 0.6282;     % Wing area in m^2
   

    % Set bounds for the optimization variables
    lb = [-40*pi/180; -0.5; 0];  % Lower bounds
    ub = [40*pi/180; 0.5; 1];    % Upper bounds

    % Use fmincon to minimize the cost function
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    cost_function = @(trim_var) calculate_cost(trim_var, trim_def, aircraft_parameters);
    
    [optimal_trim_var] = fmincon(cost_function, trim_var0, [], [], [], [], lb, ub, [], options);
    
    % Get the optimized trim state and control surface vectors
    [trim_state, trim_control] = calculate_trim_state_control(optimal_trim_var, trim_def, aircraft_parameters);
end
