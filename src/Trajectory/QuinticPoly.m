function [shoulderQuint, elbowQuint, wristQuint] = QuinticPoly(invArray)

% Joint Polynomial Matrices Initialized
shoulderQuint = [];
elbowQuint = [];
wristQuint = [];

% For loop that initializes the Quintic Polynomials
for point = 2:size(invArray,1)
    
    shoulderQPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0, 0, 0, invArray(point-1,1), invArray(point,1))';
    elbowQPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0, 0, 0, invArray(point-1,2), invArray(point,2))';
    wristQPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0, 0, 0, invArray(point-1,3), invArray(point,3))';
    
    % For loop that initializes the Poses of the Robots Trajectory
    for j=1:10
        t = (j-1)*.4 +(1+4*(point-1));
        
        elbowQuint(j+1 +10*(point-2)) = quintPolyToPos(elbowQPoly, t);
        wristQuint(j+1 +10*(point-2)) = quintPolyToPos(wristQPoly, t);
        shoulderQuint(j+1+10*(point-2)) = quintPolyToPos(shoulderQPoly, t);
    end
    
end

end