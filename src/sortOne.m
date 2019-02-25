% Sort One Function
% Sorts the current ball to the correct location and returns the generated
% trajectory.
% color - color of the ball
% 1 - Green (top)
% 2 - Blue (bottom)
% 3 - Yellow (middle)
% baseSize - the size of the base
% 1 - small (left)
% 2 - large (right) 
% init - the initial/current positon of the arm
% returns the trajectory of all of the joints

function [shoulderQuint, elbowQuint, wristQuint] = sortOne(color, baseSize, init)

%% Initalization of Variables
final = [175,0,100];
initial = init';

%% Determines the End Position based on color and baseSize
% Green
if color == 1
    if baseSize == 2
        final = [254, -180, 50];
    else
        final = [254, 180, 50];
    end
    
% Yellow
elseif color == 3
    if baseSize == 2
        final = [85, -180, 50];
    else
        final = [85, 180, 50];
    end
    
    
% Blue
elseif color == 2 
    if baseSize == 2
        final = [167, -180, 50];
    else
        final = [167, 180, 50]; 
    end
else
    disp("FAILURE")
end

disp(final)

%% Trajectory Generation

% Initial Matricies
objectPoints = [];
aboveObject = [];
invArray = [];

shoulderPoints = [];
elbowPoints = [];
wristPoints = [];

if size(initial,1)> 0
 
    objectKin = ikin(initial(1,1:3));
    
    aboveObject(1,1:2) = initial(1,1:2);
    aboveObject(1,3) = 50;
    aboveObjectKin = ikin(aboveObject(1,1:3));
    
    finalDest = ikin(final);

    place = 3;
    invArray(place,1:3) = finalDest;            
    invArray(place-1,1:3) = aboveObjectKin;  
    invArray(place-2,1:3) = objectKin;      
    
end


% For loop that initializes the Quintic Polynomials
for point = 2:size(invArray,1)
    
    shoulderPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,1), invArray(point,1))';
    elbowPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,2), invArray(point,2))';
    wristPoints = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,3), invArray(point,3))';
    
    % For loop that initializes the Poses of the Robots Trajectory
    for j=1:10
        t = (j-1)*.4 +(1+4*(point-1));
        
        elbowQuint(j+1 +10*(point-2)) = quintPolyToPos(elbowPoints, t);
        wristQuint(j+1 +10*(point-2)) = quintPolyToPos(wristPoints, t);
        shoulderQuint(j+1+10*(point-2)) = quintPolyToPos(shoulderPoints, t);
    end
    
end

% Removes the first value of every Generation to keep the robot from going
% to the 0 position. 
shoulderQuint = shoulderQuint(1,2:end); 
elbowQuint = elbowQuint(1,2:end); 
wristQuint = wristQuint(1,2:end,1); 
end