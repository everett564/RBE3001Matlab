%% RBE3001 - Laboratory 3 %%

%% Initializations:
clear
clear java

%clear import;

clear classes;
vid = hex2dec('3742');
pid = hex2dec('0007');
disp (vid );
disp (pid);
javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

%% Initializations for Graphing

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

% Initialization of the 3d Plot
figure('Name', 'Current Position', 'NumberTitle', 'off')
plot3([0,0],[0,0],[0,0]);
title('Current Postition')
xlabel('Position (Encoder Ticks)')
ylabel('Position (Encoder Ticks)')
zlabel('Position (Encoder Ticks)')

%% Initialization of Variables Used
setpts = [];
xAxis = [];
TipPos = [];
sample = [];
elap = [];
elapsedTime = 0;
runTime = 128;

%%
try
    
%% Initialized Server Values    
    SERV_ID = 01;
    SERV_ID_READ = 03; % we will be talking to server ID 37 on
    SERV_ID_PID = 04;  % the Nucleo
    
    %   This is code that was commented out a while ago
    %   pp.write(SERV_ID_READ, zeros(15,1,'single'));
    %   pause(.003);
    %   returnPacket2 = pp.read(SERV_ID_READ);
    %   homePos = [];
    %   homePos(1,1:3) = returnPacket2(1:3,1);
    
%% Initialization of the PID values
   
    %Shoulder
    Kp_Shoulder=.001;
    Ki_Shoulder=.001;
    Kd_Shoulder=0.02;
    
    %Elbow
    Kp_Elbow=.0015;
    Ki_Elbow=.0025;
    Kd_Elbow=.075;
    
    %Wrist
    Kp_Wrist=.0006;
    Ki_Wrist=.0025;
    Kd_Wrist=.05;

    
    % HAD TO CHANGE THESE VALUES WHEN BORROWED OTHER ARM
    % BELOW VALUES ARE NOT CORRECT FOR OUR ARM, DELETE WHEN ARM WORKS AGAIN

    %Shoulder

    
    
    
%% Packet PID Value Initialization
    
    % Debug Statement (Leave for debugging)
    % DEBUG   = true;          % enables/disables debug prints
    % Diabled as part of lab 3
    
    % Instantiate a packet - the following instruction allocates 64
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes. 
    packet = zeros(15, 1, 'single');
    
    % Shoulder
    packet(1) = Kp_Shoulder;
    packet(2) = Ki_Shoulder;
    packet(3) = Kd_Shoulder;
    
    % Elbow
    packet(4) = Kp_Elbow;
    packet(5) = Ki_Elbow;
    packet(6) = Kd_Elbow;
    
    % Wrist
    packet(7) = Kp_Wrist;
    packet(8) = Ki_Wrist;
    packet(9) = Kd_Wrist;
    
    %   packet(11) = 1;
    %   packet(12) = homePos(1);
    %   packet(13) = homePos(2);
    %   packet(14) = homePos(3);
    
    pp.write(SERV_ID_PID,packet);
    pause(.003);
    
    packet = zeros(15, 1, 'single');
    
    % The following code generates a sinusoidal trajectory to be
    % executed on joint 1 of the arm and iteratively sends the list of
    % setpoints to the Nucleo firmware.
    % These points have not been used in a while
    % shoulder = [0, 324, -18, -404, -295, 0];
    % elbow = [0, 133.3, -75, -14, 44, 0];
    % wrist = [0, 221, 386, 79, 129, 0];
%% Cubic Polynomials

   
%     elbowPoly1 = cubePoly(1, 5, 0, 0, 7.55, 55.02);
%     wristPoly1 = cubePoly(1, 5, 0, 0, -254.75, 302.5);
%     
%     elbowPoly2 = cubePoly(5, 9, 0, 0, 55.02, 580);
%     wristPoly2 = cubePoly(5, 9, 0, 0, 302.5, -254.75);
%     
%     elbowPoly3 = cubePoly(9, 13, 0, 0, 580, 7.55);
%     wristPoly3 = cubePoly(9, 13, 0, 0, -254.75, -254.75);
%     
%     elbowPose(1) = 0;
%     wristPose(1) = 0;

    % Inverse Points Initialization
    inversePoint1 = ikin([225,50,0]);
    inversePoint2 = ikin([225,-50,0]);
    inversePoint3 = ikin([175,-50,0]);
    inversePoint4 = ikin([175,50,0]);
    inversePoint5 = ikin([200,0,100]);

    % Array of the Inverse Points
    invArray =  [inversePoint1;
                 inversePoint2; 
                 inversePoint3;
                 inversePoint4;
                 inversePoint1;
                 inversePoint5;
                 inversePoint2;
                 inversePoint5;
                 inversePoint3;
                 inversePoint5;
                 inversePoint4;
                 
                 0,0,0];
             
    % Joint Polynomial Matrices  
    shoulderPoly = [];
    elbowPoly = [];
    wristPoly = [];
    invArray = [zeros(1,3);invArray]
    
    % For loop that initializes the Cubic Polynomials
    for point = 2:size(invArray,1)
        
        shoulderPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,1), invArray(point,1))';
        elbowPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,2), invArray(point,2))';
        wristPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0,0,0, invArray(point-1,3), invArray(point,2))';
    
        % For loop that initializes the Poses of the Robots Trajectory
        for j=1:10
            t = (j-1)*.4 +(1+4*(point-1));
            
            elbowPose(j+1 +10*(point-2)) = quintPolyToPos(elbowPoly, t);
            wristPose(j+1 +10*(point-2)) = quintPolyToPos(wristPoly, t);
            shoulderPose(j+1+10*(point-2)) = quintPolyToPos(shoulderPoly, t);
        end
    
    end

    % Initialized first points
    shoulderPose(1)=0;
    elbowPose(1) = 0;
    wristPose(1) = 0;

%% Quintic Polynomials
    
    % Joint Polynomial Matrices  
    shoulderQuint = [];
    elbowQuint = [];
    wristQuint = [];
             
    % For loop that initializes the Quintic Polynomials
    for point = 2:size(invArray,1)
        
        shoulderQPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0, 0, 0, invArray(point-1,1), invArray(point,1))';
        elbowQPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0, 0, 0, invArray(point-1,2), invArray(point,2))';
        wristQPoly = quinpoly(1+4*(point-1), 5+4*(point-1), 0, 0, 0, 0, invArray(point-1,3), invArray(point,3))';
    
        % For loop that initializes the Poses of the Robots Trajectory
        for j=1:10
            t = (j-1)*.4 +(1+4*(point-1));
            
            elbowQuint(j+1 +10*(point-2)) = quintPolyToPos(elbowQPoly, t);
            wristQuint(j+1 +10*(point-2)) = quintPolyToPos(wristQPoly, t);
            shoulderQuint(j+1+10*(point-2)) = quintPolyToPos(shoulderQPoly, t);
        end
    
    end
    
    % Initializes first points
    elbowQuint(1) = 0;
    wristQuint(1) = 0;
    shoulderQuint(1) = 0;

    % Initializes final points
    elbowQuint(runTime) = 0;
    wristQuint(runTime) = 0;
    shoulderQuint(runTime) = 0;
    
end


%% THIS IS FROM ONE OF THE FIRST STEPS (REMOVE AFTER LAB IS DONE)
%     for i=1:30
%         a = 1;
%         t = (i-1)*.4 +1;
%         
%         if(mod(i,10) == 0)
%         a = a+1;    
%         end
%         elbowPose(i+1) = polyToPos(elbowPoly(a,:), t);
%         wristPose(i+1) = polyToPos(wristPoly(a,:), t);
%         shoulderPose(i+1) = polyToPos(shoulderPoly(a,:), t);
%     end
    
%     for j=1:10
%         t = (j-1)*.4 +9;
%         elbowPose(j+11) = polyToPos(elbowPoly(2,:), t);
%         wristPose(j+11) = polyToPos(wristPoly(2,:), t);
%         shoulderPose(j+11) = polyToPos(shoulderPoly(2,:), t);
%     end
%      
%      for k=1:10
%          
%         t = (k-1)*.4 +15;
%         elbowPose(k+21) = polyToPos(elbowPoly(3,:), t);
%         wristPose(k+21) = polyToPos(wristPoly(3,:), t);
%         shoulderPose(k+21) = polyToPos(shoulderPoly(3,:), t);
%      end

   
    
%     shoulderPoly1 = cubePoly(1, 5, 0, 0, 0, inversePoint1(1));
%     elbowPoly1 = cubePoly(1, 5, 0, 0, 0, inversePoint1(2));
%     wristPoly1 = cubePoly(1, 5, 0, 0, 0, inversePoint1(3));
%     
%     shoulderPoly2 = cubePoly(9, 13, 0, 0, inversePoint1(1), inversePoint2(1));
%     elbowPoly2 = cubePoly(9, 13, 0, 0, inversePoint1(2), inversePoint2(2));
%     wristPoly2 = cubePoly(9, 13, 0, 0, inversePoint1(3), inversePoint2(3));
%     
%     shoulderPoly3 = cubePoly(15, 19, 0, 0, inversePoint2(1), 0);    
%     elbowPoly3 = cubePoly(15, 19, 0, 0, inversePoint2(2), 0);
%     wristPoly3 = cubePoly(15, 19, 0, 0, inversePoint2(3), 0);
%% Initializations for while loop
    %Set Array of Values (In Encoder Ticks, I think, do not think these are
    %used)
    shoulder = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    elbow = [0, 0, 7.55, 55.02, 580, 100, 100,100,100,100];
    wrist = [0, 0, -240, 302.5,-240, 0,0,0,0,0];
    
    % Initailized Return Matrices
    ret = [];
    ret2 = [];
    ret3 = [];


    % for tea = 1:length(shoulder) (this has been here for a while,
    % consider deleting)
    i=1;
    tea=1;
    tic
    timerVal = tic;
    counter = 0;
%% While Loop 
    while tea<=runTime
        
        counterVal = tic;
        %incremtal = (single(k) / sinWaveInc); (what is this)
        
        viaPts = zeros(1, 100);
        % Send packet to the server and get the response
        
        %pp.write sends a 15 float packet to the micro controller
        if counter>=0.4
            
            packet = zeros(15, 1, 'single');
            packet(1) = shoulderQuint(tea); 
            packet(2) = elbowQuint(tea);    
            packet(3) = wristQuint(tea);
            disp(shoulderQuint(tea));
            disp(elbowQuint(tea));
            disp(wristQuint(tea));
            
            tea=tea+1;
            pp.write(SERV_ID, packet);
            pause(0.003);
            counter = 0;
            
        end
        %pause(0.003); % Minimum amount of time required between write and read
        
        
        pp.write(SERV_ID_READ, zeros(15,1,'single'));
        
        pause(0.003);
        
        returnPacket = pp.read(SERV_ID_READ); %pp.read reads a returned 15 float backet from the nucleo.
        %timerVal = tic; Probably remove this line 
        elapsedTime = toc(timerVal);
        
        TipVals = plotDaArm(returnPacket(1:3));
        elap = [elap; elapsedTime];
        TipPos = [TipPos; TipVals'];
        csvwrite('Tip Position', TipPos);
        
        setpts = [setpts; returnPacket(1:3)'];
        sample = diff(elap);
        csvwrite('Set Points', setpts);
        
        % Return Packet Initializations
        ret = [ret;returnPacket(1)];
        ret2 = [ret2;returnPacket(2)];
        ret3 = [ret3;returnPacket(3)];
        
        counter = toc(counterVal)+counter
        
        % Incrementation of i
        i= i+1;
        
%% Debug stuff
%         if DEBUG
%             disp('Sent Packet:');
%             disp(packet);
%             disp('Received Packet:');
%             disp(returnPacket);
%         end
%         
%         for x = 0:3
%             packet((x*3)+1)=0.1;
%             packet((x*3)+2)=0;
%             packet((x*3)+3)=0;
%         end
%         
%         This version will send the command once per call of pp.write
%         pp.write(02, packet);
%         pause(0.003);
%         returnPacket2=  pp.read(02);
%         this version will start an auto-polling server and read back the
%         current data
%         returnPacket2=  pp.command(65, packet);
%         
%         if DEBUG
%             %disp('Received Packet 2:');
%             %disp(returnPacket2);
%         end
%%
        toc
        pause(.003); %timeit(returnPacket) !FIXME why is this needed?
        
    end

% Time Matrix
csvwrite('Time', elap);

% retAvg=sum(ret(1:10))/10;
% ret2Avg=sum(ret2(1:10))/10;
% ret3Avg=sum(ret3(1:10))/10;

% Clear up memory upon termination
%  rep = [];
%  retE = [];
%  retW = [];
%
%  ret(1,:) = [];
%  rep = ret;
%  ret = ret(:,1:3);
%
%  plot(ret);
%  plot(retE);
%  plot(retW);

%% Clear Statements for the Graphs
clear title xlabel ylabel
close all

%% Tip Values Declarations

% Getting values of Tip Position
xTip = TipPos(:,1);
yTip = TipPos(:,2);
zTip = TipPos(:,3);

% Getting the values for Tip Velocity
xVel = diff(xTip)/diff(elap);
yVel = diff(yTip)/diff(elap);
zVel = diff(zTip)/diff(elap);

% Getting the values for Tip Acceleration
xAcc = diff(xVel(:,1))/diff(elap(1:end));
yAcc = diff(yVel(:,1))/diff(elap(1:end));
zAcc = diff(zVel(:,1))/diff(elap(1:end));

%% Joint Values Declarations

% Getting the values for the encoder values of all of the joints
% Joint Positions
shoulderPos = setpts(:,1);
elbowPos = setpts(:,2);
wristPos = setpts(:,3);

% Joint Velocities
jvel1 = diff(shoulderPos)/diff(elap);
jvel2 = diff(elbowPos)/diff(elap);
jvel3 = diff(wristPos)/diff(elap);

%% Figure 1: 3d Tip Position
% x and y coordinates of the tip plot
figure('Name', 'Tip Position in xyz plane', 'NumberTitle', 'off')

%hold on;

% Plot Function
plot3(xTip(:,1),yTip(:,1), zTip(:,1), '-');
% Plotting of the dots
%plot(225,100, 'ro');           
%plot(175,-34.28, 'ro');
xlim([0,350])
ylim([-200,200])
zlim([-50,300]) 
%hold off;
title(' Tip Position')
xlabel('mm')
ylabel('mm')
zlabel('mm')

%% Figure 2: Joint Position
% Graphs joint positions vs time
figure('Name', 'Joint Postition', 'NumberTitle', 'off')
hold on;
plot(elap, shoulderPos);
plot(elap, elbowPos);
plot(elap, wristPos);
hold off;
title('Joint Postition')
xlabel('Time (Seconds)')
ylabel('Position (Encoder Ticks)')
legend('Shoulder Position', 'Elbow Position', 'Wrist Position')

%% Figure 3: Joint Velocity
% Graphs the velocity of all of the joints
figure('Name', 'Joint Velocity', 'NumberTitle', 'off')
hold on;
plot(elap(2:end), jvel1);
plot(elap(2:end), jvel2);
plot(elap(2:end), jvel3);
hold off;
title('Joint Velocity')
xlabel('Time (Seconds)')
ylabel('Velocity (Encoder Ticks/Second')
legend('Shoulder Velocity', 'Elbow Velocity', 'Wrist Velocity')

%% Figure 4: Tip Position
% Tip Position in time of all of the variables
figure('Name', 'Tip Time', 'NumberTitle', 'off')
hold on;
plot(elap,xTip);
plot(elap,zTip);
plot(elap,yTip);
hold off;
title('Tip Time')
xlabel('Time (Seconds)')
ylabel('Position (mm)')
legend('x-Position', 'y-Position', 'z-Position')

%% Figure 5: Tip Velocity
% Tip Velocity in time of all of the variables
figure('Name', 'Tip Velocity', 'NumberTitle', 'off')
hold on;
plot(elap(2:end),xVel);
plot(elap(2:end),yVel);
plot(elap(2:end),zVel);
hold off;
title('Tip Velocity')
xlabel('Time (Seconds)')
ylabel('Velocity (mm/s)')
legend('x-Velocity', 'y-Velocity', 'z-Velocity')

%% Figure 6: Tip Accelerations
% Tip Accelerations in time of all of the variables
figure('Name', 'Tip Accelerations', 'NumberTitle', 'off')
hold on;
plot(elap(3:end),xAcc);
plot(elap(3:end),yAcc);
plot(elap(3:end),zAcc);
hold off;
title('Tip Accelerations')
xlabel('Time (Seconds)')
ylabel('Acceleration (mm/second^2)')
legend('x-Accelation', 'y-Accelation', 'z-Accelation')

%%  CSV File Statements
%  csvwrite('Return File', rep);
%  csvwrite('Plot File Shoulder', ret);
%  csvwrite('Plot File Elbow', retE);
%  csvwrite('Plot File Wrist', retW);

pp.shutdown()

%viaPts = zeros(1, 100);
toc
