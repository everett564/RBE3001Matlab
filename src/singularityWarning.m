% Singularity Function 
% q is the robots joint values in radians
% Catches if the volume of the sphere starts to become too small and says
% approaching singularity if too small. 
% returns a boolean (unused)

function M = singularityWarning(q)
    try

        [p1, p2, p3, p4, z1, z2, z3] = fwkinJacob(q(1), q(2), q(3));
        
        % Jacobian Math
        J = jacob0(q);
        J = J(1:3,:);
        Jt = J';
        ellipse = J*Jt;
        
        % gets the eigenvalues
        e = eig(ellipse);
        a = sqrt(e(1,1));
        b = sqrt(e(2,1));
        c = sqrt(e(3,1));
        
        % Volume Formula for a Ellipse
        V = (4/3)*pi*a*b*c;
        
        if V <= 10000000 % LOL I KNOW THIS VALUE IS LARGE BUT IT WORKS
            throw exception
        end
        M = false;
        
    catch
        error('Approaching Singularity');
        M = true;
    end
end