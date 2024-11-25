function gc = guidance_circle_track(x,Va_c, R, orbit_center, direction, orbit_gains)

    % Extract aircraft state variables
    pn = x(1);  % North position
    pe = x(2);  % East position
    pd = -x(3);  % Down position (height)
    % Va = x(4);  % Airspeed
    % chi = x(5); % Course angle
    % h = x(6);   % Height

    % Circle center and parameters
    cn = orbit_center(1);  % North center of the circle
    ce = orbit_center(2);  % East center of the circle
    cd = orbit_center(3);  % Down (height) of the circle center

    % Desired height is constant
    hc = -cd;
    h_dot_c = 0;  % No change in height

    % Compute position error in the horizontal plane
    e_n = pn - cn;
    e_e = pe - ce;

    % Distance from the circle center
    d = sqrt(e_n^2 + e_e^2);

    % Calculate the commanded course angle (tangent to the circle)
    % Use arctangent to find the angle to the center, then add 90 degrees for the tangent
    phi_c = atan2(e_e, e_n);

    chi_c = phi_c + direction*(pi/2) + atan(orbit_gains.kr*(d-R)/R);

    % Calculate rate of change of course angle (proportional control)
    % K_chi = 1.0;  % Gain for heading control
    chi_dot_c = 0*direction * (Va_c/R);

    % Commanded airspeed
    %Va_c = 18;  % Constant airspeed

    % Return guidance commands
    gc = [hc, 0, chi_c, chi_dot_c, Va_c]';

end
