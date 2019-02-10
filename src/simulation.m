clear all
close all


figure('Name', 'Current Position', 'NumberTitle', 'off')
plot3([0,0],[0,0],[0,0]);
title('Current Postition')
xlabel('Position x')
ylabel('Position y')
zlabel('Position z')
view([0,0])

qi = [0,0,0];

while 1
    [x,z] = ginput;
    
    pd = [x,0,z];
    
    T = JacobInverse(qi,pd)
    
    [p1, p2, p3, p4, Z1, Z2, Z3] = fwkinJacob(T(1),T(2),T(3)); 
    
    % plot3 plots the arm in 3D
    plot3([0, p2(1), p3(1), p4(1)], [0, p2(2), p3(2), p4(2)], [0, p2(3), p3(3), p4(3)],'-or');
    
    xlim([0,350])
    ylim([-200,200])
    zlim([-50,300]) 
    
    qi = T;
end