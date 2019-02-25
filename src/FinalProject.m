%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RBE3001 - FINAL PROJECT - TEAM 3 %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Camera Initialization of Matricies
% Robot origin to checherboard origin =
originToCheck =   [1 ,  0   ,0  , 275.8;
    0 ,  1  , 0  , 113.6;
    0 ,  0,  1   ,0;
    0  , 0 ,  0  , 1];

% Camera origin to checkerboard origin =
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

% Path for all of the folders made
addpath('Kinematics','Kinematics/Inverse','Kinematics/Differential','Kinematics/Forward','CameraCalibration','Trajectory','Plotting','ObjectDetection','Other');

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
    
    SERV_ID = 01;      % Sending Server
    SERV_ID_READ = 03; % Reading Server
    SERV_ID_PID = 04;  % PID Server
    SERV_ID_GRIP = 05; % Gripper Server
    
    %% Initialization of the PID values
    
    % Shoulder
    Kp_Shoulder=.001;
    Ki_Shoulder=.001;
    Kd_Shoulder=0.02;
    
    % Elbow
    Kp_Elbow=.0015;
    Ki_Elbow=.0025;
    Kd_Elbow=.075;
    
    % Wrist
    Kp_Wrist=.0008;
    Ki_Wrist=.0025;
    Kd_Wrist=.05;
    
    
    %% Packet PID Value Initialization
    
    % Instantiate a packet
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
    
    % Write these Values
    pp.write(SERV_ID_PID,packet);
    pause(.003);
    
    packet = zeros(15, 1, 'single');
end

%% Initializations for while loop

% Initailized Return Matrices
ret = [];
ret2 = [];
ret3 = [];

% Initialized Values
i=1;
tea=1;
tic;
timerVal = tic;
counter = 0;
counter2=0;
lastState = 0;

% State Logic
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
                % Reset the tea value
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

pp.shutdown()
toc
