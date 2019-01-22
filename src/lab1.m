%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.
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
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 
f = figure
figure = plot([0,0],[0,0])
hold on
xlim([0,1000])
ylim([-1000,1000])
xAxis = [];
try
  SERV_ID = 01; 
  SERV_ID_READ = 03;% we will be talking to server ID 37 on
  SERV_ID_PID = 04;% the Nucleo
%   
%   pp.write(SERV_ID_READ, zeros(15,1,'single'));
%   pause(.003);
%   returnPacket2 = pp.read(SERV_ID_READ);
%   homePos = [];
%   homePos(1,1:3) = returnPacket2(1:3,1);

  
  
  
  Kp_Shoulder=.001;
  Ki_Shoulder=.001;
  Kd_Shoulder=0.02;
  
  
  Kp_Elbow=.003;
  Ki_Elbow=.0025;
  Kd_Elbow=.04;
  
  Kp_Wrist=.0006;
  Ki_Wrist=.0025;
  Kd_Wrist=.05;
  
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
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
  
  
  shoulder = [0, 100,0, 0,0, 0];
  elbow = [0, 0, 0, 0, 0, 0];
  wrist = [0, 0, 0, 0, 0, 0];
  

  ret = [];
  ret2 = [];

  % Iterate through a sine wave for joint values
  
  %for tea = 1:length(shoulder)
  i=1;
  tea=1;
  
  while tea<=6
     
      tic
      %incremtal = (single(k) / sinWaveInc);
      
      
      

     viaPts = zeros(1, 100);
      % Send packet to the server and get the response
      
      %pp.write sends a 15 float packet to the micro controller
       if mod(i,166)==0
        packet = zeros(15, 1, 'single');
        packet(1) = shoulder(tea);
        packet(2) = elbow(tea);
        packet(3) = wrist(tea);
        tea=tea+1;
        pp.write(SERV_ID, packet); 
       end
       pause(0.003); % Minimum amount of time required between write and read
       
       %pp.read reads a returned 15 float backet from the nucleo.
       pp.write(SERV_ID_READ, zeros(15,1,'single'));
       pause(0.003);
       returnPacket = pp.read(SERV_ID_READ);
       
       ret = [ret;returnPacket(1)];
       ret2 = [ret2;returnPacket(2)];
       xAxis = [xAxis;i];
       i= i+1;
%         xAxis(i,1) = i;
        set(figure, 'Xdata', xAxis');
        set(figure, 'Ydata', ret);
        %drawnow
        %set(figure, 'Ydata', ret2);
        %plot(x(1:i),ret(1:i))
        
        drawnow
       toc
      
       
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
      %THis version will send the command once per call of pp.write
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
      pause(.003) %timeit(returnPacket) !FIXME why is this needed?
      
  end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
%  rep = [];
%  retE = [];
%  retW = [];
%  
%  ret(1,:) = [];
%  rep = ret;
%  ret = ret(:,1:3);
%  
%  
%  plot(ret);
% plot(retE);
 %plot(retW);
 
%  csvwrite('Return File', rep);
%  csvwrite('Plot File Shoulder', ret);
%  csvwrite('Plot File Elbow', retE);
%  csvwrite('Plot File Wrist', retW);

pp.shutdown()

%viaPts = zeros(1, 100);
toc
