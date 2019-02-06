% Forward Kinematics Function 
% fwkin3001 does the forward Kinematics of the Arm
% Parameters: Takes in three encoder values (in ticks) 
% t1 - the encoder value of the shoulder
% t2 - the encoder value of the elbow
% t3 - the encoder value of the wrist
% return: This function returns a matrix of three positions in radians

function [P1, P2, P3, P4] = fwkin(t1,t2,t3)
    
    % Converts the encoder values from ticks to radians
    t1 = -(t1*6.28)/4096; 
    t2 =  (t2*6.28)/4096;
    t3 =  (t3*6.28)/4096;
    
    % Length of each arm
    l1 = 135;    % Link 1
    l2 = 175;    % Link 2
    l3 = 169.28; % Link 3
    
    % T01 (Transformation Matrix from base to shoulder)
    
    % constants obtained from the DH table
    alpha1 = 0;
    d1 = 0;
    a1 = 0;
    
    T01 = [cos(t1), -sin(t1)*cos(alpha1), sin(t1)*sin(alpha1), a1*cos(t1);
           sin(t1), cos(t1)*cos(alpha1), -cos(t1)*sin(alpha1), a1*sin(t1);
           0, sin(alpha1), cos(alpha1), d1;
           0,0,0,1];
    
    % T12 (Transformation Matrix from shoulder to elbow)
    
    % constants obtained from the DH table
    alpha2 = pi/2;
    d2 = l1;
    a2 = 0;
    
    T12 = [cos(t2), -sin(t2)*cos(alpha2), sin(t2)*sin(alpha2), a2*cos(t2);
           sin(t2), cos(t2)*cos(alpha2), -cos(t2)*sin(alpha2), a2*sin(t2);
           0, sin(alpha2), cos(alpha2), d2;
           0,0,0,1];
    
    % T23 (Transformation Matrix from elbow to wrist)
    
    % constants obtained from the DH table
    alpha3 = 0;
    d3 = 0;
    a3 = l2;
    
    
    T23 = [cos(t3), -sin(t3)*cos(alpha3), sin(t3)*sin(alpha3), a3*cos(t3);
           sin(t3), cos(t3)*cos(alpha3), -cos(t3)*sin(alpha3), a3*sin(t3);
           0, sin(alpha3), cos(alpha3), d3;
           0,0,0,1];
       
   
    % T34 (Transformation Matrix from wrist to gripper)
    
    % constants obtained from the DH table
    alpha4 = 0;
    d4 = 0;
    a4 = l3;    
    
    T34 = [cos(-pi/2), -sin(-pi/2)*cos(alpha4), sin(-pi/2)*sin(alpha4), a4*cos(-pi/2);
           sin(-pi/2), cos(-pi/2)*cos(alpha4), -cos(-pi/2)*sin(alpha4), a4*sin(-pi/2);
           0, sin(alpha4), cos(alpha4), d4;
           0,0,0,1];
    
    % Produced Matrices from other Transformation Matricies
    T04 = T01*T12*T23*T34;
    T03 = T01*T12*T23;
    T02 = T01*T12;
    
    % Grabbing the position from the produced matricies
    P4 = T04(1:3, 4);
    P3 = T03(1:3, 4);
    P2 = T02(1:3, 4);
    P1 = T01(1:3, 4);
   
    
end