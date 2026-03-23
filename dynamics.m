
function xdot = dynamics(~, x, J, torque_ext)
 
     % Vector part of quaternion
     epsilon = x(1:3);
     % Scalar part of quaternion
     eta = x(4); 
     % Angular velocity
     omega = x(5:7);   
    
    %Quarternions
    epsilon_cross = skew(epsilon);   
    epsilon_dot = 0.5 * (eta * eye(3) + epsilon_cross) * omega;
    eta_dot = -0.5 * (epsilon' * omega);
     
    %J*omega_dot = torque_ext - omega x (J * omega)
    omega_dot = inv(J) \ (torque_ext - cross(omega, J * omega));
    
    %complete xdot vector
    xdot = [epsilon_dot; eta_dot; omega_dot];

end


%skew symmetric matrix function
function a = skew(b)
    a = [  0,   -b(3),  b(2);
          b(3),   0,   -b(1);
         -b(2),  b(1),   0  ];
end