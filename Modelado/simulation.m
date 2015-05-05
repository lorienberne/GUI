clc;
close all;
clear all;
tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PARAMETERS DECLARATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% SIMULATION PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TIME IN SECONDS BETWEEN EACH TIME STEP
dt = 0.01;


%% KALMAN FILTER PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MCov        = 0.002^2;
% ACov        = 0.0004^2;
% GCov        = (0.5*pi/180)^2;
% GPSPosCov   = 1;
% GPSSpeedCov = 0.1;

MCov        = 0.0001;
ACov        = 0.0001;
GCov        = 0.0001;
GPSPosCov   = 1;
GPSSpeedCov = 0.1;
GPSDirCov   = 0.001;
BCov        = 0.1;
%% KALMAN ATTITUDE MODEL
% COVARIANCE MATRIX
attQ = [MCov 0 0 0 0 0;
        0 ACov 0 0 0 0; % A -> ACCELEROMETER'S COVARIANCE
        0 0 ACov 0 0 0;
        0 0 0 GCov 0 0;
        0 0 0 0 GCov 0;
        0 0 0 0 0 GCov];

% COVARIANCE MATRIX
attR = [MCov 0 0 0 0 0;
        0 ACov 0 0 0 0; % A -> ACCELEROMETER'S COVARIANCE
        0 0 ACov 0 0 0;
        0 0 0 GCov 0 0;
        0 0 0 0 GCov 0;
        0 0 0 0 0 GCov];

% KALMAN MATHEMATICAL MODEL MATRICES
attA = [0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1; 
        0 0 0 0 0 0;
        0 0 0 0 0 0; 
        0 0 0 0 0 0];   
attB = [     0         0         0;
             0         0         0;
             0         0         0;
      0.000212         0         0;
             0     0.000212      0;
             0         0    0.00003638];
attC = eye(6);
attD = zeros(6,3);


%% KALMAN ORIENTATION MODEL
% COVARIANCE MATRIX

psiQ = [MCov 0;
       0 GCov];
% COVARIANCE MATRIX
psiR = [MCov 0;
        0 GCov];
    
% KALMAN MATHEMATICAL MODEL MATRICES
psiA = [0 1;
        0 0];
psiB = [0;0.00003638];
psiC = eye(2);
psiD = zeros(2,1);

%% KALMAN HORIZONTAL POSITION MODEL
% COVARIANCE MATRIX
posQ = [GPSPosCov   0 0 0 0 0;
        0 GPSPosCov   0 0 0 0;
        0 0 GPSPosCov   0 0 0;
        0 0 0 GPSSpeedCov 0 0;
        0 0 0 0 GPSSpeedCov 0;
        0 0 0 0 0 GPSSpeedCov];

% COVARIANCE MATRIX
posR = [GPSPosCov   0 0 0 0 0;
        0 GPSPosCov   0 0 0 0;
        0 0 GPSPosCov   0 0 0;
        0 0 0 GPSSpeedCov 0 0;
        0 0 0 0 GPSSpeedCov 0;
        0 0 0 0 0 GPSSpeedCov];

% KALMAN MATHEMATICAL MODEL MATRICES
posA = [0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0];    


posB = [0          0         0     0;
        0          0         0     0;
        0          0         0     0;
        0          0      -96.145  0;
        0        96.145      0     0;
  -0.0000094838    0         0     0];

posC = eye(6);
posD = zeros(6,4);

%% KALMAN ALTITUDE MODEL
% COVARIANCE MATRIX
zQ = [GPSPosCov 0
      0 GPSSpeedCov];
  
% COVARIANCE MATRIX
zR = [GPSPosCov 0;
      0 GPSSpeedCov];

% KALMAN MATHEMATICAL MODEL MATRICES
zA = [0 1;
      0 0];

zB = [0;-0.0000094838];
zC = eye(2);
zD = zeros(2,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% QUAD OBJECT PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% HALF DISTANCE BETWEEN EACH PAIR OF PROPELLER'S AXES
l = 0.1;

% MASS OF THE QUADCOPTER
m = 1.05;

% DIAGONAL ELEMENTS OF THE INERTIA TENSOR OF THE QUADCOPTER
I = [0.0128 0.0128 0.0256];

% INTERTIA MOMENT OF INERTIA OF THE PROPELLER
rotorIz = 0.0001;

% THRUST AND TORQUE CONSTANTS
kProp = [0.000009958, 0.0000009315];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%% FEEDBACK PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

attK =  [1.9707   0     0  0.9067   0     0  ;
            0  1.9707   0    0   0.9067   0  ;
            0     0  6.5484  0      0  3.0127]*1000000;

posK =  [         0           0      -1000000       0               0         -4592200  ;
                  0      0.1*0.8*1.5    0           0          0.10103*1.5*1.3    0     ;
             -0.1*0.8*1.5     0         0      -0.10103*1.5*1.3     0             0     ;
                  0           0         0           0               0             0     ];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% INITIAL CONDITIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ATTITUDE STATE VECTOR (PHI THETA PSI PHIDOT THETADOT PSIDOT)
attitSttVect = [0; 0; 0; 0; 0; 0];

% POSITION STATE VECTOR (X Y Z XDOT YDOT ZDOT)
posSttVect   = [0; 0; 0; 0; 0; 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% OBJECT INSTANTIATION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%luenObs = luenberger(K) %UNCOMMENT TO USE A LUENBERGER OBSERVER
attKalFil       = kalman(attQ, attR, attA, attB, attC, attD, dt); %COMMENT IF YOU USE A LUENBERGER OBSERVER
psiKalFil       = kalman(psiQ, psiR, psiA, psiB, psiC, psiD, dt);
posKalFil       = kalman(posQ, posR, posA, posB, posC, posD, dt);
zKalFil         = kalman(zQ, zR, zA, zB, zC, zD, dt);
q           	= quad(posSttVect, attitSttVect, m, l, I, rotorIz, kProp, dt);
attfback        = fback(attK);
posfback        = fback(posK);
accelSensor     = accel(ACov);
magnetSensor    = magnet(MCov);
gyroSensor      = gyro(GCov);
gpsSensor       = gps([GPSPosCov; GPSSpeedCov; GPSDirCov]);
baroSensor      = barometer(BCov);
abFilter        = alphabeta();
posNmedFilter   = nmedidas(10,3,dt);
attNmedFilter   = nmedidas(10,3,dt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


posDesState = [2; 3; -3; 0; 0; 0];
posFBSignal = [0; 0; 0; 0];

% VARIABLES TO SAVE INFORMATION FOR LATER PLOTS
plotPosReal    = [];
plotPosMed     = [];
plotPosKalFil  = [];
plotPosAlphaFil= [];
plotPosNMes    = [];

attPosReal    = [];
attPosMed     = [];
attPosKalFil  = [];
attPosAlphaFil= [];
attPosNMes    = [];

plotAttV       = [];
plotPosV       = [];

plotPosSignal  = [];
plotAttSignal  = [];


for t = 0:dt:5
%GET INPUT
  rotorOmega = q.getRotorOmega();
  attU = [             rotorOmega(1)^2 - rotorOmega(3)^2;
                       rotorOmega(4)^2 - rotorOmega(2)^2;
     rotorOmega(1)^2 + rotorOmega(3)^2 - (rotorOmega(4)^2 + rotorOmega(2)^2)];

  posU = posFBSignal;

%GET SENSOR MEASUREMENTS
  attMState = [q.attitSttVect(1:2)
               magnetSensor.magMeasure(q);
               gyroSensor.gyroMeasure(q)];
  psiMState =  gpsSensor.getGPSDir(q);
  posMState =  gpsSensor.gpsMeasure(q);
  zMState   =  baroSensor.baroMeasure(q);
  
%ESTIMATE QUAD STATE
  attKalFil   = attKalFil.updateKalman(attU, attMState);
  psiKalFil   = psiKalFil.updateKalman(attU(3), psiMState);
  posKalFil   = posKalFil.updateKalman(posU, posMState); 
  zKalFil     = zKalFil.updateKalman(posU(3),zMState);
  attKalState = attKalFil.getState(); 
  posKalState = posKalFil.getState();
  psiKalState = psiKalFil.getState();
  zKalState   = zKalFil.getState();
  
%FILTER WITH ALPHA BETA FILTER
 sigmaGPS = posKalFil.pPosteriori();
 sigmaAtt = attKalFil.pPosteriori();
 sigmaPsi = psiKalFil.pPosteriori(); 
 sigmaZ   = zKalFil.pPosteriori();
 zAlphaFiltered   = abFilter.alphaFilter(sigmaGPS(3,3),sigmaZ(1,1),posKalState(3),zKalState(1));
 dirAlphaFiltered = abFilter.alphaFilter(sigmaAtt(3,3),sigmaPsi(1,1),attKalState(3),psiKalState(1)); 

%REBUILD POSITION AND ATTITUDE STATE VECTORS
  attState = [attKalState(1:2)
              dirAlphaFiltered;
              attKalState(4:6)];
              
  posState = [posKalState(1:2);
              zAlphaFiltered;
              posKalState(4:6)];
  
%UPDATE N MEASUREMENTS FILTER
 posNmedFilter = posNmedFilter.nfilter(posState(1:3), posState(4:6));
 attNmedFilter = attNmedFilter.nfilter(attState(1:3), attState(4:6));
 
%GET AVEARAGE MEASUREMENTS FROM N MEASUREMENTS FILTER
 attNFil = attNmedFilter.getNMes();
 posNFil = posNmedFilter.getNMes();

%GET CONTROL SIGNAL AND SET PROPS TO THAT SPEED
  posFBSignal = posfback.getControlSignal(posNFil, posDesState);
  attFBSignal = attfback.getControlSignal(attNFil,[posFBSignal(2:4,1); 0; 0; 0]);
  q = q.setPropSpeed(attFBSignal,posFBSignal(1,1));
    
%CARRY OUT A SIMULATION TIMESTEP
  q = q.simQuad();
%PLOT THE RESULT
  q.drawQuad(3);
  pause(dt);
  

%SAVE DATA FOR PLOTS
plotPosReal    = [plotPosReal q.posSttVect(1:3)];
plotPosMed     = [plotPosMed posMState];
plotPosKalFil  = [plotPosKalFil posKalState(1:3)];
plotPosAlphaFil= [plotPosAlphaFil posState(1:3)];
plotPosNMes    = [plotPosNMes posNFil(1:3)];
end


% t = 0:dt:5;
% 
% figure();
% hold on;
% plot(t,plotPosReal(3,:));
% plot(t,plotPosMed(3,:));
% plot(t,plotPosKalFil(3,:));
% plot(t,plotPosAlphaFil(3,:));
% plot(t,plotPosNMes(3,:));
% legend('Real','Medida','FiltroKalman','FiltroAB','FiltroNMed');
% hold off;


toc;