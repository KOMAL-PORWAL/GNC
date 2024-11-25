% Function to convert air-relative velocity vector in body coordinates to wind angles
% Komal Porwal
% Created: September 5, 2024

function [wind_angles] = AirRelativeVelocityVectorToWindAngles(velocity_body)
    % This function takes air relative components in body coordinates and returns wind angles in column vector 

    u = velocity_body(1); % Body axis X component
    v = velocity_body(2); % Body axis Y component
    w = velocity_body(3); % Body axis Z component

    Va = sqrt(u^2 + v^2 + w^2); % Total air-relative velocity magnitude

    % Calculate angle of attack (alpha) and sideslip angle (beta) in
    % radians
    alpha = atan2(w, u);
    beta = asin(v / Va);

    % Output the wind angles as a column vector
    wind_angles = [Va; beta; alpha];
end
