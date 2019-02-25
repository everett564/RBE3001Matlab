% Jacobian Function
% jacob0 makes the jacobian matrix
% Parameters: Takes in three encoder values (in radians) 
% q - the matrix of values for the joints
% return: This function returns a matrix of the Jacobian


function J = jacob0(q)
    
    %Obtains the fwkin for the Jacobian
    [p1, p2, p3, pe, z1, z2, z3] = fwkinJacob(q(1),q(2),q(3));
    
    % J1
    J1(1:3,1) = cross(z1,(pe-p1));
    J1(4:6,1) = z1;
    
    % J2
    J2(1:3,1) = cross(z2,(pe-p2));
    J2(4:6,1) = z2;
    
    %J3
    J3(1:3,1) = cross(z3,(pe-p3));
    J3(4:6,1) = z3;
    
    % return
    J = [J1, J2, J3];
    
end