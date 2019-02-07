% Forward Kinematics Function for the Jacobian
% fwkinJacob does the forward Kinematics of the Arm
% Parameters: Takes in three encoder values (in radians) 
% t1 - the shoulder value 
% t2 - the elbow value
% t3 - the wrist value
% return: This function returns a matrix of 7 values needed to calculate a Jacobian

function [P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(t1,t2,t3)
       
    % Length of each arm
    l1 = 135;    % Link 1
    l2 = 175;    % Link 2
    l3 = 169.28; % Link 3
    
    T01 = tdh(0,t1,0,0);       % base to shoulder
    T12 = tdh(l1,0,0,pi/2);    % shoulder to elbow    
    T23 = tdh(0,t2,l2,0);      % elbow to wrist
    T34 = tdh(0,t3-pi/2,l3,0); % wrist to gripper
    
    % Produced Matrices from other Transformation Matricies
    T04 = T01*T12*T23*T34;
    T03 = T01*T12*T23;
    T02 = T01*T12;
    
    % Grabbing the values from the produced matricies
    P4 = T04(1:3, 4);
    P3 = T03(1:3, 4);
    P2 = T02(1:3, 4);
    P1 = T01(1:3, 4);
    Z3 = T03(1:3, 3);
    Z2 = T02(1:3, 3);
    Z1 = T01(1:3, 3);

end