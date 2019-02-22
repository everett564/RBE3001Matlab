function [shoulderQuint, elbowQuint, wristQuint] = sortOne(color, baseSize, initial) 
final = [175,0,100];
% This is where we have it make the trajectory variables

% Green
if size(ismember(color,'green')) ==0
    if size(ismember(baseSize,'large')) ==0
        % qf = location to be sorted (254, -160, -10)
        final = [254, -160, 100];
    else
        % qf = location to be sorted (254, 160, -10)
        final = [254, 160, 100];
    end


% Yellow
elseif size(ismember(color,'yellow')) ==0
    if size(ismember(baseSize,'large')) ==0
        % qf = location to be sorted (85, -160, -10)
        final = [85, -160, 100];
    else
        % qf = location to be sorted (85, 160, -10)
        final = [85, 160, 100];
    end


% Blue
else %if size(ismember(color,'blue')) ==0
    if size(ismember(baseSize,'large')) ==0
       % qf = location to be sorted (167, -160, -10) 
       final = [167, -160, 100];
    else
       % qf = location to be sorted (167, 160, -10)
       final = [167, 160, 100];
    end
end


% This is where we make the trajectory
invArray = initial'; 
invArray = [invArray; final];
[shoulderQuint, elbowQuint, wristQuint] = QuinticPoly(invArray);

end