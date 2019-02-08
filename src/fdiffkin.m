% Forward Differential Kinematics Formula
% fdiffkin calculates the foward differential kinematics
% q is the vector of the current joint variables 
% qdiff is the joint velocities
% return returns the task space velocities pder

function pder = fdiffkin(q,qder)
    
    % Find the Jacobian of q    
    J = jacob0(q);
    
    % Multiply the Jacobian by the derivative of q
    % return
    pder = J*qder';
    
end