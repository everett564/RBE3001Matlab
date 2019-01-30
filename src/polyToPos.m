function q = polyToPos(a, t)
    %takes in 4x1 array of polynomial coefficients a and a given time t
    %and puts out the target location q
    
    %coefficients in input a 
    a0 = a(1);
    a1 = a(2);
    a2 = a(3);
    a3 = a(4);
    
    q = (a0 + a1*t + a2*(t^2) + a3*(t^3))*(2*pi/4096);
    q;
    
end