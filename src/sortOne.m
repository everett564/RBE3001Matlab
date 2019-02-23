function [shoulderQuint, elbowQuint, wristQuint] = sortOne(color, baseSize, init)
final = [175,0,100];
initial = init';
% This is where we have it make the trajectory variables


%% This logic is broken
% Green
if color ==1
    if baseSize ==2
        % qf = location to be sorted (254, -160, -10)
        final = [254, -180, 50];
    else
        % qf = location to be sorted (254, 160, -10)
        final = [254, 180, 50];
    end
    
    
    % Yellow
elseif color == 3
    if baseSize  ==2
        % qf = location to be sorted (85, -160, -10)
        final = [85, -180, 50];
    else
        % qf = location to be sorted (85, 160, -10)
        final = [85, 180, 50];
    end
    
    
    % Blue
else %if size(ismember(color,'blue')) ==0
    if baseSize == 2
        
        % qf = location to be sorted (167, -160, -10)
        final = [167, -180, 50];
    else
        % qf = location to be sorted (167, 160, -10)
        final = [167, 180, 50]; % always gets to this, never any other
    end
end

disp(final)
%% This is where we make the trajectory

objectPoints = [];
aboveObject = [];
invArray = [];

% robotFrame pose is the initial conditions 
% 
% [0,0,50] -> hover above object -> go to object
if size(initial,1)> 0
    
%     objectPoints(1,1:2) = inital(1,1:2);
%     objectPoints(1,3) = -10;
    objectKin = ikin(initial(1,1:3));
    
    aboveObject(1,1:2) = initial(1,1:2);
    aboveObject(1,3) = 50;
    aboveObjectKin = ikin(aboveObject(1,1:3));
    
    finalDest = ikin(final); % this needs to be the end location
    
    
    place = 3;
    invArray(place,1:3) = finalDest;            
    invArray(place-1,1:3) = aboveObjectKin;  
    invArray(place-2,1:3) = objectKin;      
    
end

shoulderPoints = [];
elbowPoints = [];
wristPoints = [];


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

% invArray = initial';
% invArray = [invArray; final];
% [shoulderQuint, elbowQuint, wristQuint] = QuinticPoly(invArray);
shoulderQuint = shoulderQuint(1,2:end); 
elbowQuint = elbowQuint(1,2:end); 
wristQuint = wristQuint(1,2:end,1); 
end