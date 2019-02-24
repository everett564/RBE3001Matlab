function B = findNearestBase (ballPose, matOfBaseLoc,sizeOut)
    N = matOfBaseLoc(1,:);
    index = 1;
    
    for i=2:size(matOfBaseLoc, 1)
        disp(N)
       if distMat(ballPose, matOfBaseLoc(i,:)) < distMat(ballPose, N)
           N = matOfBaseLoc(i,:);
           index = i;
       end
    end
    
    B = sizeOut(index);
end