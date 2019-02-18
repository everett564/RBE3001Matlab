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

while 1
imgOrg = snapshot(cam);

[imOutput, robotFramePose, diskDia] = findObjs(imgOrg, checkToOrigin, camToCheck, cParams);


end