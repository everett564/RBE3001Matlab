% Poly to Position Function
% polyToPos takes in 4x1 array of polynomial coefficients a and a given time 
% Parameters :
% a: matrix of coefficients
% t: time
% return: puts out the target location q

function q = polyToPos(a, t)
    
    % Coefficients in input a 
    a0 = a(1);
    a1 = a(2);
    a2 = a(3);
    a3 = a(4);
    
    % Formula
    q = (a0 + a1*t + a2*(t^2) + a3*(t^3))*(4096/(2*pi));
    q;
    
end