function M = singularityWarning(q)
    try
%         J = J(1:3,:);
%         V = det(J);

        [p1, p2, p3, p4, z1, z2, z3] = fwkinJacob(q(1), q(2), q(3));
        
        J = jacob0(q);
        J = J(1:3,:);
        Jt = J.';
        ellipse = J*Jt;
        [a,b,c] = ellipse_values(ellipse, [p4(1),p4(2),p4(3)]);
        
        V = (4/3)*pi*a*b*c;
        
        if V <= 1
            throw exception
        end
        M = false;
    catch
        error('Approaching Singularity');
        M = true;
    end
end