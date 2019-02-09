function [a,b,c] = ellipse_values(A, centre)
    x = A(:,1);
    y = A(:,2);
    z = A(:,3);
    
    i = 1;
    while i < 4
        x(i) = sqrt(x(i,1)-centre(i));
        y(i) = sqrt(y(i,1)-centre(i));
        z(i) = sqrt(z(i,1)-centre(i));
        i=i+1;
    end
    
    a = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
    b = sqrt(y(1)^2 + y(2)^2 + y(3)^2);
    c = sqrt(z(1)^2 + z(2)^2 + z(3)^2);
    
end
