% Jacobian Inverse Function
% This is step 9 in Lab 4
% qi is the initial joint positions (1x3)
% pd is the final positon (1x3)
% returns the final joint angles

function T = JacobInverse(qi, pd)

% Makes the Plot
figure('Name', 'Current Position', 'NumberTitle', 'off') 
plot3([0,0],[0,0],[0,0]);
title('Current Postition')

% Sets the View
view([0,0])

% Gets the position of the end effector from the forward kinematics
[P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(qi(1),qi(2),qi(3)); 

while norm(pd - (P4')) > 0  
    
%calculates the position every time it runs through the loop    
[P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(qi(1),qi(2),qi(3));    
    
% Gets the Jacobian
J = jacob0(qi);
A = J(1:3,1:3);
inJ = inv(A);    

delq = (pd -(P4'))*inJ;

qi = qi + delq;

[p1, p2, p3, p4, Z1, Z2, Z3] = fwkinJacob(qi(1),qi(2),qi(3));
    
% plot3 plots the arm in 3D
plot3([0, p2(1), p3(1), p4(1)], [0, p2(2), p3(2), p4(2)], [0, p2(3), p3(3), p4(3)],'-or');

xlim([0,350])
ylim([-200,200])
zlim([-50,300])

%used to show the iterations it makes in the graph
pause(.001);

end

% return
T = qi;

end