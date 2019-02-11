% Jacobian Inverse Function
% Moves the arm from one position to the next. 
% Parameters: qi and pd
% qi is the initial position matrix (1x3)
% pd is the final position (1x3)
% returns a (1x3) matrix of the joint angles

function T = JacobInverse(qi, pd)

% Makes the Plot
figure(1);
plot3([0,0],[0,0],[0,0]);
title('Current Postition')

% Gets the position of the end effector from the forward kinematics
[P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(qi(1),qi(2),qi(3)); 

while norm((pd - (P4.'))) > 0.001  
    
% Calculates the position every time it runs through the loop    
[P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(qi(1),qi(2),qi(3));    
    
% Gets the Jacobian
J = jacob0(qi);
A = J(1:3,1:3);
inJ = inv(A);    

delq = inJ*(pd.' - P4);

qi = qi + delq.';

[p1, p2, p3, p4, Z1, Z2, Z3] = fwkinJacob(qi(1),qi(2),qi(3));
    
% plot3 plots the arm in 3D
plot3([0, p2(1), p3(1), p4(1)], [0, p2(2), p3(2), p4(2)], [0, p2(3), p3(3), p4(3)],'-or');

% Sets the View
view([0,0])

% Sets the limits
xlim([0,350])
ylim([-200,200])
zlim([0,300])

% Used to show the iterations it makes in the graph
pause(.001);

end

disp(qi);

% Return
T = qi;

end