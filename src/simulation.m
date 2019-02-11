close all

% Initial configuration for the plot
qi = [0,0,0];

while 1
    
    figure(1)
    xlim([0,350])
    ylim([0,200]) % this is actually the z value

    % gets the input
    [x,z] = ginput;
   
    pd = [x,0,z];
    
    % Runs this function to modify the graph that is used 
    T = JacobInverse(qi,pd);
    
    % Sets the old position as the new start position
    qi = pd;
    
    % Displays the Angles from the Jacob Inverse Function
    disp(T);
end
