function M = singularityWarning(q)
    try
%         J = J(1:3,:);
%         V = det(J);

        [p1, p2, p3, p4, z1, z2, z3] = fwkinJacob(q(1), q(2), q(3));
        
        J = jacob0(q);
        J = J(1:3,:);
        Jt = J';
        ellipse = J*Jt;
        
        e = eig(ellipse);
        a = sqrt(e(1,1));
        b = sqrt(e(2,1));
        c = sqrt(e(3,1));
        
        V = (4/3)*pi*a*b*c;
        
        if V <= 1000
            throw exception
        end
        M = false;
    catch
        error('Approaching Singularity');
        M = true;
    end
end