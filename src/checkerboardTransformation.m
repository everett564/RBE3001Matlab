% takes in an a nx2 array of coordinates (x,y) in camera reference frame
function [ret] = checkerboardTransformation(pixelPoints)

cameraParams = camCal();

originToCheck= [1 ,  0 ,  0  , 275.8;
                0 ,  1 ,  0  , 113.6;
                0 ,  0 ,  1  , 0;
                0 ,  0 ,  0  , 1];
 
camToCheck= [-0.0017 , -0.8032 ,   0.5957,  107.6207;
              0.9998  ,  0.0094  ,  0.0155 , 109.2884;
             -0.0180  ,  0.5956  ,  0.8031 , 277.1416;
              0 ,        0   ,      0,       1.0000];

originToCam = originToCheck*(camToCheck^-1);
 
R = originToCam(1:3,1:3);
t = originToCam(1:3, 4);

% Testing Points for Step 4
% pixelPoints = [282, 297;
%                282, 396;
%                156, 397;
%                409, 396];

worldPoints = pointsToWorld(cameraParams, R, t, pixelPoints);

ret = worldPoints;

end

