% Function to transform a vector from inertial coordinates to body coordinates
% Komal Porwal
% Created on: 30-Aug-2024

function vector_body = TransformFromInertialToBody(vector_inertial, euler_angles)
    % This function transforms a vector from inertial coordinates to body coordinates
    % using the provided Euler angles.
    

    % Calculation of the 3-2-1 rotation matrix using the Euler angles
    R = RotationMatrix321(euler_angles);

    % Transform the vector from inertial to body coordinates
    vector_body = R * vector_inertial;

    % Output the transformed vector
end
