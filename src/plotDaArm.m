% Plotting the Arm Function
% plotDaArm plots the 3d version of our arm
% Parameters: takes in a matrix of three encoder values in ticks
% q - matrix of three encoder values
% return: returns the end position of the arm
% can also be used without a return

function T = plotDaArm(q)
    
    % does forward kinematics of the joints
    [p1 p2 p3] = fwkin3001(q(1), q(2), q(3));
    
    % plot3 plots the arm in 3D
    plot3([0, p1(1), p2(1), p3(1)], [0, p1(2), p2(2), p3(2)], [0, p1(3), p2(3), p3(3)],'-or');
    
    % sets the limits of the graph
    xlim([0,350])
    ylim([-200,200])
    zlim([-50,300]) 
    
    %return used for tip position
    T = p3;

end