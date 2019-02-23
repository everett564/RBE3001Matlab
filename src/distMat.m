function d = distMat(ballPose, basePose)
    d = sqrt((basePose(1)-ballPose(1))^2 + (basePose(2)-ballPose(2))^2);
end