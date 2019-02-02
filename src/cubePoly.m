% Cubic Polynomial Function
% cubePoly does the cubic polynomial of the arm
% Parameters: takes in 6 varaibles
% t0 - Initial time
% tf - Final time
% v0 - Initial Velocity
% vf - Final Velocity
% q0, qf - Positions
% returns the cubic polynomial matrix


function T = cubePoly(t0, tf, v0, vf, q0, qf)
    
    %Variables in a Cubic Polynomial
    vars = [1, t0, (t0)^2, (t0)^3;
            0, 1, 2*t0, 3*(t0^2);
            1, tf, (tf)^2, (tf)^3;
            0, 1, 2*tf, 3*(tf^2)];
        
    %Constants in the Cubic Polynomial
    cons = [q0;
            v0;
            qf;
            vf];
        
    %This is what we want to return
    T = (vars^-1)*cons;
    
end