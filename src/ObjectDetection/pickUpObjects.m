% Pick Up Objects Function
% ob - the object number
% Returns
% shoulder - shoulder trajectory
% elbow - elbow trajectory
% wrist - wrist trajectory
% invArray - the points the robot is going to
% colors - the color of the object
% colorAndBase - the color and base matrix of all bases and colors detected

function [shoulder, elbow, wrist, invArray, colors, colorAndBase] = pickUpObjects(ob)

%% Camera Intialization
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

imgOrg = snapshot(cam);

[imOutput, robotFramePose, colorAndBase, colors] = findObjs(imgOrg, checkToOrigin, camToCheck, cParams);

objectPoints = [];
aboveObject = [];
invArray = [];

%% Path Generation
if size(robotFramePose,1)> 0
    objectPoints(ob,1:2) = robotFramePose(ob,1:2);
    objectPoints(ob,3) = -20;
    objectKin = ikin(objectPoints(ob,1:3));
    
    aboveObject(ob,1:2) = robotFramePose(ob,1:2);
    aboveObject(ob,3) = 50;
    aboveObjectKin = ikin(aboveObject(ob,1:3));
    
    hover = ikin([175,0,50]);
    
    place = (3*ob);
    invArray(place-2,1:3) = hover;
    invArray(place-1,1:3) = aboveObjectKin;
    invArray(place,1:3) = objectKin;
else
    hover = ikin([175,0,50]);
    invArray(1,1:3) = hover;
    invArray(2,1:3) = hover;
end

shoulderPoints = [];
elbowPoints = [];
wristPoints = [];

%% Quintic Polynomials Initialization from the invArray generated 
for point = 2:size(invArray,1)
    
    shoulderPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,1), invArray(point,1))';
    elbowPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,2), invArray(point,2))';
    wristPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,3), invArray(point,3))';
    
    % For loop that initializes the Poses of the Robots Trajectory
    for j=1:10
        t = (j-1)*.4 +(1+4*(point-1));
        
        elbow(j+1 +10*(point-2)) = quintPolyToPos(elbowPoints, t);
        wrist(j+1 +10*(point-2)) = quintPolyToPos(wristPoints, t);
        shoulder(j+1+10*(point-2)) = quintPolyToPos(shoulderPoints, t);
    end
    
end

% Removes the first 0 of every matrix
shoulder = shoulder(1,2:end);
elbow = elbow(1,2:end);
wrist = wrist(1,2:end,1);

end
