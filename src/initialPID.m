%Initialization of PID Values Function
% Used to save space in main code

function T = initialPID ()
  
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
  
end