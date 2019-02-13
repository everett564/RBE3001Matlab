cameraParams = camCal();

originToCheck=   [1 ,  0   ,0  , 275.8;
0 ,  1  , 0  , 113.6;
0 ,  0,  1   ,0;
0  , 0 ,  0  , 1];
%
%
% camera origin to checkerboard origin =
% 
camToCheck=   [-0.0017 ,  -0.8032 ,   0.5957,  107.6207;
0.9998  ,  0.0094  ,  0.0155 , 109.2884;
-0.0180  ,  0.5956  ,  0.8031 , 277.1416;
     0 ,        0   ,      0,    1.0000];
 originToCam = originToCheck/camToCheck;
 
 R = originToCam(1:3,1:3);
 t = originToCam(1:3, 4);
 
 points = [143, 379;
     369, 384;
     164, 244;
     412, 304];
    
 
worldPoints = pointsToWorld(cameraParams, R, t, points);
worldPoints = [worldPoints [0;0;0;0] [1;1;1;1]];

robotPoints = worldPoints/originToCheck

