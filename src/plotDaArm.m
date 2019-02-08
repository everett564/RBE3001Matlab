% Plotting the Arm Function
% plotDaArm plots the 3d version of our arm
% Parameters: takes in a matrix of three encoder values in ticks
% q - matrix of three encoder values
% return: returns the end position of the arm
% can also be used without a return

function T = plotDaArm(q,qder)
    
    q(1) = -(q(1)*6.28)/4096;
    q(2) = (q(2)*6.28)/4096;
    q(3) = (q(3)*6.28)/4096;
    
    % does forward kinematics of the joints
    [p1, p2, p3, p4, z1, z2, z3] = fwkinJacob(q(1), q(2), q(3));
    
    % plot3 plots the arm in 3D
    plot3([0, p2(1), p3(1), p4(1)], [0, p2(2), p3(2), p4(2)], [0, p2(3), p3(3), p4(3)],'-or');
    grid on
    
    % Velocity Vector Plot
    velocityArr = fdiffkin(q,qder);
    hold on
    quiver3(p4(1),p4(2),p4(3),velocityArr(2)*.005,velocityArr(1)*.005,velocityArr(3)*.005);
    hold off
    % sets the limits of the graph
    xlim([0,350])
    ylim([-200,200])
    zlim([-50,300]) 
    
    %return used for tip position
    T = p4;

end