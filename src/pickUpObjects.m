function [shoulder, elbow, wrist] = pickUpObjects()

    clear all
    close all

    cam = webcam();
    cParams = camCal();

    checkToOrigin= [-1 ,  0 ,  0  , 275.8;
                    0 ,  1 ,  0  ,  113.6;
                    0 ,  0 ,  -1  ,     0;
                    0 ,  0 ,  0  , 1];

    camToCheck= [-0.0017 , -0.8032 ,   0.5957,  107.6207;
                  0.9998  ,  0.0094  ,  0.0155 , 109.2884;
                 -0.0180  ,  0.5956  ,  0.8031 , 277.1416;
                  0 ,        0   ,      0,       1.0000];

    checkToCam = camToCheck^-1;
    %checkToOrigin = originToCheck^-1;

    imgOrg = snapshot(cam);

    [imOutput, robotFramePose, diskDia] = findObjs(imgOrg, checkToOrigin, camToCheck, cParams);
    
    objectPoints = [];
    invArray = [];
    
    for object = 1:size(robotFramePose,1)
        objectPoints(object,1:2) = robotFramePose(object,1:2);
        objectPoints(object,3) = 0;
        objectKin = ikin(objectPoints(object,1:3));
        invArray(object,1:3) = objectKin;
    end
    
    shoulderPoints = [];
    elbowPoints = [];
    wristPoints = [];
    invArray = [zeros(1,3);invArray;zeros(1,3)];

    % For loop that initializes the Cubic Polynomials
    for point = 2:size(invArray,1)

        shoulderPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,1), invArray(point,1))';
        elbowPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,2), invArray(point,2))';
        wristPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,3), invArray(point,2))';

        % For loop that initializes the Poses of the Robots Trajectory
        for j=1:10
            t = (j-1)*.4 +(1+4*(point-1));

            elbow(j+1 +10*(point-2)) = quintPolyToPos(elbowPoints, t);
            wrist(j+1 +10*(point-2)) = quintPolyToPos(wristPoints, t);
            shoulder(j+1+10*(point-2)) = quintPolyToPos(shoulderPoints, t);
        end

    end

    % Initialize first points
    shoulder(1)=0;
    elbow(1) = 0;
    wrist(1) = 0;

end
