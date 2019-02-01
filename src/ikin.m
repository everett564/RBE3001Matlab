% Inverse Kinematics Function
% ikin does the inverse kinematics of the position
% Parameters: takes in a matrix of the x y and z in radians(i think)
% returns the inverse kinematics matrix of the angles in radians

function T = ikin(q)
try
    %Initialization
    x=q(1);
    y=q(2);
    z=q(3);
    
    % Lengths of the links of the arm in mm
    L1 = 135;
    L2 = 175;
    L3 = 169.28;
    
    n=sqrt(x^2 + y^2);
    
    alpha = atan2((z-L1),n);
    
    beta = acos((L2^2 + n^2 + (z-L1)^2 -L3^2)/(2*L2*sqrt(n^2 + (z-L1)^2)));
    
    elbowTheta = alpha + beta;
    wristTheta = acos(-1*(L2^2 + L3^2 -(n^2 + (z-L1)^2))/(2*L2*L3)) - pi/2;
    shoulderTheta = atan2(y,x);
    
    % Bounds
    if (shoulderTheta < -.92)||(shoulderTheta > .92)||(elbowTheta > 1.53)||(elbowTheta < -.09)||(wristTheta < -.314)||(wristTheta > .92)
        throw(exception)
    end
    
    % Return matrix
    T = [shoulderTheta, elbowTheta, wristTheta];

catch
    error('Outside of Bounds');
end