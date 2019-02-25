% Find Nearest Base Function
% Finds the nearest base to the balls position
% ballPose - the balls position (1)(x) and (2)(y)
% matOfBaseLoc - the matrix of all of the base locations
% sizeOut - the matrix of all of the base Sizes
% Returns the size of the nearest base

function B = findNearestBase (ballPose, matOfBaseLoc, sizeOut)
    
    % Initial Conditions
    N = matOfBaseLoc(1,:);
    index = 1;
    
    % Iterates through the rest of the bases
    for i=2:size(matOfBaseLoc, 1)
        disp(N)
       if distMat(ballPose, matOfBaseLoc(i,:)) < distMat(ballPose, N)
           N = matOfBaseLoc(i,:);
           index = i;
       end
    end
    
    % Return 
    B = sizeOut(index);
end