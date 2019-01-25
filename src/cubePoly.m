function T = cubePoly(t0, tf, v0, vf, q0, qf)
    
    vars = [];
    vars = [1, t0, (t0)^2, (t0)^3;
            0, 1, 2*t0, 3*(t0^2);
            1, tf, (tf)^2, (tf)^3;
            0, 1, 2*tf, 3*(tf^2)];
    cons = [q0;
            v0;
            qf;
            vf];
        
        
        T = (vars^-1)*cons;
    
end