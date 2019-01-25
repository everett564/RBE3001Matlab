
function [P1 P2 P3] = fwkin3001(t1,t2,in3)
    t1 = -(t1*6.28)/4096; % keion says this is 4096
    t2 = (t2*6.28)/4096;
    in3 = (in3*6.28)/4096;
    

    l1 = 135;
    l2 = 175;
    l3 = 169.28;
    
    % T01
    % constants obtained from the table we made
    alpha1 = pi/2;
    d1 = l1;
    a1 = 0;
    
    T01 = [cos(t1), -sin(t1)*cos(alpha1), sin(t1)*sin(alpha1), a1*cos(t1);
        sin(t1), cos(t1)*cos(alpha1), -cos(t1)*sin(alpha1), a1*sin(t1);
        0, sin(alpha1), cos(alpha1), d1;
        0,0,0,1];
    
    % T12
    % constants obtained from the table we made
    alpha2 = 0;
    d2 = 0;
    a2 = l2;
    
    T12 = [cos(t2), -sin(t2)*cos(alpha2), sin(t2)*sin(alpha2), a2*cos(t2);
        sin(t2), cos(t2)*cos(alpha2), -cos(t2)*sin(alpha2), a2*sin(t2);
        0, sin(alpha2), cos(alpha2), d2;
        0,0,0,1];
    
    % T23
    % constants obtained from the table we made
    alpha3 = 0;
    d3 = 0;
    a3 = l3;
    t3 = in3 -pi/2;
    
    
    T23 = [cos(t3), -sin(t3)*cos(alpha3), sin(t3)*sin(alpha3), a3*cos(t3);
        sin(t3), cos(t3)*cos(alpha3), -cos(t3)*sin(alpha3), a3*sin(t3);
        0, sin(alpha3), cos(alpha3), d3;
        0,0,0,1];
    
    % matricies we need
    T03 = T01*T12*T23;
    T02 = T01*T12;
    
    % grabbing the position from the produced matricies
    P3 = T03(1:3, 4);
    P2 = T02(1:3, 4);
    P1 = T01(1:3, 4);
   
    
    
    
end