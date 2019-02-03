% Quintic Polynomial Function
% quinpoly does the quintic polynomial of the arm
% Parameters: takes in 6 varaibles
% t0 - Initial time
% tf - Final time
% v0 - Initial Velocity
% vf - Final Velocity
% a0 - Initial Acceleration
% af - Final Acceleration
% q0, qf - Positions
% returns the cubic polynomial matrix

function T = quinpoly(t0, tf, v0, vf, a0, af, q0, qf)
    
    %Variables in a Quintic Polynomial
    vars = [1, t0,(t0)^2, (t0)^3,(t0)^4,(t0)^5;
            0, 1, 2*t0, 3*(t0)^2, 4*(t0)^3, 5*(t0)^4;
            0, 0, 2, 6*t0, 12*(t0)^2, 20*(t0)^3;
            1, tf,(tf)^2,(tf)^3,(tf)^4,(tf)^5;
            0, 1, 2*tf, 3*(tf)^2, 4*(tf)^3, 5*(tf)^4;
            0,0,2,6*tf, 12*(tf)^2, 20*(tf)^3];
    
    %Constants in the Quintic Polynomial    
    cons = [q0;
            v0;
            a0;
            qf;
            vf;
            af];
        
    %This is what we want to return
    T = (vars^-1)*cons;
end