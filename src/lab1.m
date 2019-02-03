%%
% RBE3001 - Laboratory 3
%

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

%%

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

% Initialization of the 3d Plot
figure('Name', 'Current Position', 'NumberTitle', 'off')
plot3([0,0],[0,0],[0,0]);
title('Current Postition')
xlabel('Position (Encoder Ticks)')
ylabel('Position (Encoder Ticks)')
zlabel('Position (Encoder Ticks)')

%Initialization of Variables Used:
setpts = [];
xAxis = [];
TipPos = [];
elap = [];
elapsedTime = 0;

try
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
%%    
    
    % Debug Statement (Leave for debugging)
    DEBUG   = true;          % enables/disables debug prints
    
    % Instantiate a packet - the following instruction allocates 64
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes. (CONSIDER MAKING TO A FUNCTION)
    packet = zeros(15, 1, 'single');
    
    packet(1) = Kp_Shoulder;
    packet(2) = Ki_Shoulder;
    packet(3) = Kd_Shoulder;
    
    packet(4) = Kp_Elbow;
    packet(5) = Ki_Elbow;
    packet(6) = Kd_Elbow;
    
    packet(7) = Kp_Wrist;
    packet(8) = Ki_Wrist;
    packet(9) = Kd_Wrist;
    %
    %   packet(11) = 1;
    %   packet(12) = homePos(1);
    %   packet(13) = homePos(2);
    %   packet(14) = homePos(3);
    %
    
    pp.write(SERV_ID_PID,packet);
    pause(.003);
    
    packet = zeros(15, 1, 'single');
    
    % The following code generates a sinusoidal trajectory to be
    % executed on joint 1 of the arm and iteratively sends the list of
    % setpoints to the Nucleo firmware.
    %shoulder = [0, 324, -18, -404, -295, 0];
    %elbow = [0, 133.3, -75, -14, 44, 0];
    %wrist = [0, 221, 386, 79, 129, 0];
    
%% Cubic Polynomials
   
    elbowPoly1 = cubePoly(1, 5, 0, 0, 7.55, 55.02);
    wristPoly1 = cubePoly(1, 5, 0, 0, -254.75, 302.5);
    
    elbowPoly2 = cubePoly(5, 9, 0, 0, 55.02, 580);
    wristPoly2 = cubePoly(5, 9, 0, 0, 302.5, -254.75);
    
    elbowPoly3 = cubePoly(9, 13, 0, 0, 580, 7.55);
    wristPoly3 = cubePoly(9, 13, 0, 0, -254.75, -254.75);
    
    elbowPose(1) = 0;
    wristPose(1) = 0;
    
    for i=1:10
        t = (i-1)*.4 +1;
        elbowPose(i+1) = polyToPos(elbowPoly1, t);
        wristPose(i+1) = polyToPos(wristPoly1, t);
    end
    
    for j=1:10
        t = (j-1)*.4 +5;
        elbowPose(j+11) = polyToPos(elbowPoly2, t);
        wristPose(j+11) = polyToPos(wristPoly2, t);
    end
    
    for k=1:10
        t = (k-1)*.4 +9;
        elbowPose(k+21) = polyToPos(elbowPoly3, t);
        wristPose(k+21) = polyToPos(wristPoly3, t);
    end
    
    elbowPose(32) = 0;
    wristPose(32) = 0;
    
%%
    %Set Array of Values (In Encoder Ticks)
    shoulder = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    elbow = [0, 0, 7.55, 55.02, 580, 100, 100,100,100,100];
    wrist = [0, 0, -240, 302.5,-240, 0,0,0,0,0];
    
    %Initailized Return Matrices
    ret = [];
    ret2 = [];
    ret3 = [];
    
    %inversePoint1 = ikin([225,120,100]);
    %inversePoint2 = ikin([100,-120,50]);
    
    
    %for tea = 1:length(shoulder)
    i=1;
    tea=1;
    tic
    timerVal = tic;
    counter = 0;
    while tea<=32
        
        counterVal = tic;
        %incremtal = (single(k) / sinWaveInc);
        
        viaPts = zeros(1, 100);
        % Send packet to the server and get the response
        
        %pp.write sends a 15 float packet to the micro controller
        if counter>=0.4
            packet = zeros(15, 1, 'single');
            packet(1) = shoulder(tea)*(4096/2*pi); % shoulder is in encoder ticks
            packet(2) = elbowPose(tea)*(4096/2*pi);
            packet(3) = wristPose(tea)*(4096/2*pi);
            tea=tea+1;
            pp.write(SERV_ID, packet);
            counter = 0;
        end
        pause(0.003); % Minimum amount of time required between write and read
        
        %pp.read reads a returned 15 float backet from the nucleo.
        pp.write(SERV_ID_READ, zeros(15,1,'single'));
        pause(0.003);
        returnPacket = pp.read(SERV_ID_READ);
        %timerVal = tic;
        elapsedTime = toc(timerVal);
        
        plotDaArm(returnPacket(1:3))
        TipVals = plotDaArm(returnPacket(1:3));
        elap = [elap; elapsedTime];
        TipPos = [TipPos; TipVals'];
        csvwrite('Tip Position', TipPos);
        
        setpts = [setpts; returnPacket(1:3)'];
        csvwrite('Set Points', setpts);
        
        ret = [ret;returnPacket(1)];
        ret2 = [ret2;returnPacket(2)];
        ret3 = [ret3;returnPacket(3)];
        
        
        counter = toc(counterVal)+counter
        
        % xAxis = [xAxis;i];
        i= i+1;
        % xAxis(i,1) = i;
        % set(figure, 'Xdata', xAxis');
        % set(figure, 'Ydata', ret);
        %drawnowap = [elap; elapsedtime];
        %set(figure, 'Ydata', ret2);
        %plot(x(1:i),ret(1:i))
        
        %drawnow
        
        
        if DEBUG
            disp('Sent Packet:');
            disp(packet);
            disp('Received Packet:');
            disp(returnPacket);
        end
        
        for x = 0:3
            packet((x*3)+1)=0.1;
            packet((x*3)+2)=0;
            packet((x*3)+3)=0;
        end
        
        %This version will send the command once per call of pp.write
        %pp.write(02, packet);
        %pause(0.003);
        %returnPacket2=  pp.read(02);
        %this version will start an auto-polling server and read back the
        %current data
        %returnPacket2=  pp.command(65, packet);
        
        if DEBUG
            %disp('Received Packet 2:');
            %disp(returnPacket2);
        end
        toc
        pause(.003); %timeit(returnPacket) !FIXME why is this needed?
        
    end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end
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

clear title xlabel ylabel
close all

% Getting values of Tip Position
xTip = TipPos(:,1);
yTip = TipPos(:,2);
zTip = TipPos(:,3);

% Getting the values for the encoder values of all of the joints
shoulderPos = setpts(:,1);
elbowPos = setpts(:,2);
wristPos = setpts(:,3);

% Joint Velocities
jvel1 = diff(shoulderPos)/diff(elap);
jvel2 = diff(elbowPos)/diff(elap);
jvel3 = diff(wristPos)/diff(elap);

% Figure 1: Tip Time
% Tip Position in time of all of the variables
figure('Name', 'Tip Time', 'NumberTitle', 'off')
hold on;
plot(elap,xTip);
plot(elap,zTip);
plot(elap,yTip);
hold off;
title('Tip Time')
xlabel('Time (Seconds)')
ylabel('Position (Encoder Ticks)')
legend('x-Position', 'y-Position', 'z-Position')

% Figure 2: Tip Position
% x and y coordinates of the tip
figure('Name', 'Tip Position', 'NumberTitle', 'off')
hold on;
plot(xTip, zTip, '-o');
plot(191.2566,122.9888, 'ro');
plot(112.3549,-20.2409, 'ro');
plot(262.5972,5.2783, 'ro');
hold off;
title(' Tip Position')
xlabel('Position (Encoder Ticks)')
ylabel('Position (Encoder Ticks)')

% Figure 3: Joint Position
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

% Figure 4: Joint Velocity
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

%  Files Unused at the moment
%  csvwrite('Return File', rep);
%  csvwrite('Plot File Shoulder', ret);
%  csvwrite('Plot File Elbow', retE);
%  csvwrite('Plot File Wrist', retW);

pp.shutdown()

%viaPts = zeros(1, 100);
toc
