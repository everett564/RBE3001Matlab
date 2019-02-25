% Simulation Script
% Used for Simulating Lab 4 step 9
% will display the angles produced from the function Jacob Inverse

close all

% Initial configuration for the plot
qi = [0,0,0];
% Number of Times the Loop Runs
runs = 6;

for i=1:runs
    
    figure(2)
    xlim([0,350])
    ylim([0,200]) % this is actually the z value

    % gets the input
    [x,z] = ginput;
   
    % final position from the input
    pd = [x,0,z];
    
    % Runs this function to modify the graph that is used 
    R = JacobInverse(qi,pd);
    
    % Sets the old position as the new start position
    qi = pd;
    
    R = R(1, 1:3);
    
    % Displays the Angles from the Jacob Inverse Function
    disp(R);
    
end
