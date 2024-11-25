    % Problem 1 c)  
    % Komal Porwal
    % Created on 11th September 2024
function [xdot] = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

    % Unpack state vector
    x = aircraft_state(1);  % x position in inertial frame
    y = aircraft_state(2);  % y position in inertial frame
    z = aircraft_state(3);  % z position in inertial frame
    phi = aircraft_state(4);    % Roll angle
    theta = aircraft_state(5);  % Pitch angle
    psi = aircraft_state(6);    % Yaw angle
    euler_angles = aircraft_state(4:6);
    velocity_body = aircraft_state(7:9);
    u = aircraft_state(7);      % Body frame x velocity
    v = aircraft_state(8);      % Body frame y velocity
    w = aircraft_state(9);      % Body frame z velocity
    p = aircraft_state(10);     % Roll rate
    q = aircraft_state(11);     % Pitch rate
    r = aircraft_state(12);     % Yaw rate
    %time = 100;

    rho = stdatmo(-z);

    % Inertia and mass
    m = aircraft_parameters.m; %[kg]
    g = aircraft_parameters.g;
    SLUGFT2_TO_KGM2 = 14.5939/(3.2804*3.2804);
    aircraft_parameters.Ix = SLUGFT2_TO_KGM2*4106/12^2/32.2; %[kg m^2]
    aircraft_parameters.Iy = SLUGFT2_TO_KGM2*3186/12^2/32.2; %[kg m^2]
    aircraft_parameters.Iz = SLUGFT2_TO_KGM2*7089/12^2/32.2; %[kg m^2]
    aircraft_parameters.Ixz = SLUGFT2_TO_KGM2*323.5/12^2/32.2; %[kg m^2]

    %% Angular accelerations in the body frame
    % Inertia matrix
    % %I_body = [aircraft_parameters.Ix, 0, -aircraft_parameters.Ixz;
    %           0,  aircraft_parameters.Iy, 0;
    %          -aircraft_parameters.Ixz, 0,  aircraft_parameters.Iz];

    I_body = aircraft_parameters.inertia_matrix;

    % Get forces and moments from AircraftForcesAndMoments
    [aero_force, aero_moment] = AircraftForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, rho, aircraft_parameters);
    % Rotational dynamics (Euler's equations for rotational motion)

    R = RotationMatrix321(euler_angles);
    angular_velocity_body = [p; q; r];
    angular_acceleration_body = I_body \ (aero_moment - cross(angular_velocity_body, I_body * angular_velocity_body));

    % Translational dynamics (Newton's second law in body frame)
    acceleration_body = (aero_force / m) - (cross(angular_velocity_body, aircraft_state(7:9)));  % [ax, ay, az] in body frame

    % Convert body frame acceleration to inertial frame (using rotation matrix)
    acceleration_inertial = TransformFromBodyToInertial(acceleration_body, euler_angles);
    
   

    % Derivatives of Euler angles (based on angular rates)
    phi_dot = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    theta_dot = q * cos(phi) - r * sin(phi);
    psi_dot = (q * sin(phi) + r * cos(phi)) / cos(theta);

    % Construct xdot (time derivatives of the state vector)
    xdot = zeros(12, 1);  % Preallocate the derivative vector

    % Inertial position derivatives (dx/dt = u_inertial, dy/dt = v_inertial, dz/dt = w_inertial)
    xdot(1:3) = TransformFromBodyToInertial(velocity_body, euler_angles);

    % Euler angle derivatives (dphi/dt, dtheta/dt, dpsi/dt)
    xdot(4) = phi_dot;
    xdot(5) = theta_dot;
    xdot(6) = psi_dot;

    % Body frame velocity derivatives (du/dt, dv/dt, dw/dt)
    xdot(7:9) = acceleration_body;

    % Angular velocity derivatives (dp/dt, dq/dt, dr/dt)
    xdot(10:12) = angular_acceleration_body;

end

