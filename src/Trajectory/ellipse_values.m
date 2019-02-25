% A is the ellipse 
%centre point of the ellipse (current tip val of the robot)

function [a,b,c] = ellipse_values(A, centre)
    e = eig(A);
    a = sqrt(e(1,1));
    b = sqrt(e(1,2));
    c = sqrt(e(1,3));
end
