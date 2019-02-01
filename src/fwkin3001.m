% Forward Kinematics Function 
% fwkin3001 does the forward Kinematics of the Arm
% Parameters: Takes in three encoder values (in ticks) 
% t1 - the encoder value of the shoulder
% t2 - the encoder value of the elbow
% t3 - the encoder value of the wrist
% return: This function returns a matrix of three positions in radians

function [P1 P2 P3] = fwkin3001(t1,t2,t3)
    
    % Converts the encoder values from ticks to radians
    t1 = -(t1*6.28)/4096; 
    t2 =  (t2*6.28)/4096;
    t3 =  (t3*6.28)/4096;
    
    % Length of each arm
    l1 = 135;    % Link 1
    l2 = 175;    % Link 2
    l3 = 169.28; % Link 3
    
    % T01 (Transformation Matrix from 0 to 1)
    
    % constants obtained from the DH table
    alpha1 = pi/2;
    d1 = l1;
    a1 = 0;
    
    T01 = [cos(t1), -sin(t1)*cos(alpha1), sin(t1)*sin(alpha1), a1*cos(t1);
           sin(t1), cos(t1)*cos(alpha1), -cos(t1)*sin(alpha1), a1*sin(t1);
           0, sin(alpha1), cos(alpha1), d1;
           0,0,0,1];
    
    % T12 (Transformation Matrix from 1 to 2)
    
    % constants obtained from the DH table
    alpha2 = 0;
    d2 = 0;
    a2 = l2;
    
    T12 = [cos(t2), -sin(t2)*cos(alpha2), sin(t2)*sin(alpha2), a2*cos(t2);
           sin(t2), cos(t2)*cos(alpha2), -cos(t2)*sin(alpha2), a2*sin(t2);
           0, sin(alpha2), cos(alpha2), d2;
           0,0,0,1];
    
    % T23 (Transformation Matrix from 2 to 3)
    
    % constants obtained from the DH table
    alpha3 = 0;
    d3 = 0;
    a3 = l3;
    t3 = t3 -pi/2;
    
    
    T23 = [cos(t3), -sin(t3)*cos(alpha3), sin(t3)*sin(alpha3), a3*cos(t3);
           sin(t3), cos(t3)*cos(alpha3), -cos(t3)*sin(alpha3), a3*sin(t3);
           0, sin(alpha3), cos(alpha3), d3;
           0,0,0,1];
    
    % Produced Matrices from other Transformation Matricies
    T03 = T01*T12*T23;
    T02 = T01*T12;
    
    % Grabbing the position from the produced matricies
    P3 = T03(1:3, 4);
    P2 = T02(1:3, 4);
    P1 = T01(1:3, 4);
   
    
end