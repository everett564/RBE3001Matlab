
function T= fwkin3001(t1,t2,in3)

    l1 = 135;
    l2 = 175;
    l3 = 169.28;

    alpha1 = pi/2;
    d1 = l1;
    a1 = 0;
    
    T03 = [];
    T01 = [cos(t1), -sin(t1)*cos(alpha1), sin(t1)*sin(alpha1), a1*cos(t1);
        sin(t1), cos(t1)*cos(alpha1), -cos(t1)*sin(alpha1), a1*sin(t1);
        0, sin(alpha1), cos(alpha1), d1;
        0,0,0,1];
    
    alpha2 = 0;
    d2 = 0;
    a2 = l2;
    
    T12 = [cos(t2), -sin(t2)*cos(alpha2), sin(t2)*sin(alpha2), a1*cos(t2);
        sin(t2), cos(t2)*cos(alpha2), -cos(t2)*sin(alpha2), a1*sin(t2);
        0, sin(alpha2), cos(alpha2), d2;
        0,0,0,1];
    
    alpha3 = 0;
    d3 = 0;
    a3 = l3;
    t3 = in3 -pi/2;
    
    
    T23 = [cos(t3), -sin(t3)*cos(alpha3), sin(t3)*sin(alpha3), a1*cos(t3);
        sin(t3), cos(t3)*cos(alpha3), -cos(t3)*sin(alpha3), a1*sin(t3);
        0, sin(alpha3), cos(alpha3), d3;
        0,0,0,1];
    
    T03 = T01*T12*T23;
    T02 = T01*T12;
    P3 = T03(4,1:3);
    P2 = T02(4,1:3);
    P1 = T01(4,1:3);
end