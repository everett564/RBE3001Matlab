% Jacobian Inverse Function
% This is step 9 in Lab 4
% qi is the initial joint positions
% pd is the final positon
% returns the final joint angles

function T = JacobInverse(qi,pd)

%Initialization
q0 = [0,0,0];

q0 = qi;

[P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(q0(1),q0(2),q0(3)); 

while (pd - (P4')) > 0

% Forward Kinematics 
[P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(q0(1),q0(2),q0(3));    
    
% Gets the Jacobian
J = jacob0(q0)
    
delq = inv(J)*(pd - (P4'));

q0 = q0 + delq;

end

% return
T = q0;

end