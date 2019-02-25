% Distance between two points in 2D Space Function
% ballPose - the balls position (1)(x) and (2)(y)
% Returns the distance between the matricies

function d = distMat(ballPose, basePose)
    
    % Return
    d = sqrt((basePose(1)-ballPose(1))^2 + (basePose(2)-ballPose(2))^2);
    
end