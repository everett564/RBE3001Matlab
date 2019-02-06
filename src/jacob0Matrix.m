function J = jacob0Matrix(q)
    [p1, p2, p3, pe] = fwkin(q(1),q(2),q(3));
    [z1, z2, z3, useless] = fwkinJacobZ(q(1),q(2),q(3));
    
    J1(1:3,1) = cross(z1,(pe-p1));
    J1(4:6,1) = z1;
    
    J2(1:3,1) = cross(z2,(pe-p2));
    J2(4:6,1) = z2;
    
    J3(1:3,1) = cross(z3,(pe-p3));
    J3(4:6,1) = z3;
    
    J = [J1, J2, J3];
    
end