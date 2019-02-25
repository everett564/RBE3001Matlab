%% RBE3001 - FINAL PROJECT%%
%% Camera Initializations
% robot origin to checherboard origin =
originToCheck =   [1 ,  0   ,0  , 275.8;
    0 ,  1  , 0  , 113.6;
    0 ,  0,  1   ,0;
    0  , 0 ,  0  , 1];

% camera origin to checkerboard origin =
camToCheck=   [-0.0017 ,  -0.8032 ,   0.5957,  107.6207;
    0.9998  ,  0.0094  ,  0.0155 , 109.2884;
    -0.0180  ,  0.5956  ,  0.8031 , 277.1416;
    0 ,        0   ,      0,    1.0000];
%% Initializations:
clear
clear java
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
addpath('Kinematics/Inverse','Kinematics/Differential','Kinematics/Forward','CameraCalibration','Trajectory','Plotting','ObjectDetection','Other');

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
TipVels = [0;0;0];
TipVals = [0,0,0];
TipValSave = [0,0,0];

try
    
    %% Initialized Server Values
    SERV_ID = 01;
    SERV_ID_READ = 03; % we will be talking to server ID 37 on
    SERV_ID_PID = 04;  % the Nucleo
    SERV_ID_GRIP = 05; % Gripper Channel
    
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
    Kp_Wrist=.0008;
    Ki_Wrist=.0025;
    Kd_Wrist=.05;
    
    
    %% Packet PID Value Initialization
    
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
    
    pp.write(SERV_ID_PID,packet);
    pause(.003);
    
    packet = zeros(15, 1, 'single');
end

%% Initializations for while loop
% Set Array of Values (In Encoder Ticks, I think, do not think these are
% used)
shoulder = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
elbow = [0, 0, 7.55, 55.02, 580, 100, 100,100,100,100];
wrist = [0, 0, -240, 302.5,-240, 0,0,0,0,0];

% Initailized Return Matrices
ret = [];
ret2 = [];
ret3 = [];

i=1;
tea=1;
tic;
timerVal = tic;
counter = 0;
counter2=0;
lastState = 0;

searching = 1;
sorting = 2;
moving = 3;
state = searching;

%% While Loop
while 1
    
    % Searching State
    if state == searching
        pp.write(SERV_ID_GRIP, 1);
        
        [objectShoulder, objectElbow, objectWrist, deleteMe, colors, colorAndBase] = pickUpObjects(1);
        
        if size(colorAndBase)>0
        state = moving;
        lastState = searching;
        end
       
    end
    
    
    % Sorting State
    if state == sorting
        
        rp1 = returnPacket(1)*((2*pi)/4096);
        rp2 = returnPacket(2)*((2*pi)/4096);
        rp3 = returnPacket(3)*((2*pi)/4096);
        
        [P1, P2, P3, P4, Z1, Z2, Z3] = fwkinJacob(rp1,rp2,rp3);
        
        colorAndBase=colorAndBase(all(colorAndBase,2),:); 
        disp(colorAndBase)
        P4 = [P4(1); -P4(2); P4(3)];
        
        [objectShoulder, objectElbow, objectWrist] = sortOne(colorAndBase(1,1), colorAndBase(1,2), P4);
        
        state = moving;
        lastState = sorting;
    end
    
    % Moving State
    if state == moving
        
        if counter>=0.4
            
            if tea <= size(objectShoulder,2)

                radianShoulder = objectShoulder(tea) * ((2*pi)/4096);
                radianElbow = objectElbow(tea) * ((2*pi)/4096);
                radianWrist = objectWrist(tea) * ((2*pi)/4096);
                
                singularityWarning([radianShoulder, radianElbow, radianWrist]);
                
                % Makes the packets
                packet = zeros(15, 1, 'single');
                packet(1) =objectShoulder(tea);
                packet(2) = objectElbow(tea);
                packet(3) = objectWrist(tea);
                
                tea=tea+1;
                pp.write(SERV_ID, packet);
                pause(0.003);
                counter = 0;
                
                
            else
                tea = 1;
                
                % We have reached the object
                if lastState == searching
                    
                    pp.write(SERV_ID_GRIP, 0);
                    state = sorting;
                    lastState = moving;
                    
                else
                    
                    state = searching;
                    pp.write(SERV_ID_GRIP, 1);
                    
                end
                
            end
        end
        
    end
    
    counterVal = tic;
    viaPts = zeros(1, 100);
    
    % Where the packets are written and sent
    pp.write(SERV_ID_READ, zeros(15,1,'single'));
    
    pause(0.003);
    
    % pp.read reads a returned 15 float backet from the nucleo.
    returnPacket = pp.read(SERV_ID_READ);
    
    
    elapsedTime = toc(timerVal);
    
    counter2 = toc(counterVal)+counter2;
    
    TipVals = plotDaArm(returnPacket(1:3),TipVels');
    
    if counter2 >=.1
        TipVels = TipVals - TipValSave;
        TipVels = TipVels/counter2;
        TipValSave = TipVals;
        counter2=0;
    end
     
    % Return Packet Initializations
    ret = [ret;returnPacket(1)];
    ret2 = [ret2;returnPacket(2)];
    ret3 = [ret3;returnPacket(3)];
    
    counter = toc(counterVal)+counter;
    
    % Incrementation of i
    i= i+1;
    
    toc
    pause(.003); 
    
end

% Time Matrix
csvwrite('Time', elap);

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
