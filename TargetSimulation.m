%-----------------------------------------------------------
%% MAIN: Data Simulation
% Simulate backscattering data of moving radar target objects in 77.6GHz.
% Targets:
% 1. Pedestrians
% 2. Bycicles
% 3. Cars
%-----------------------------------------------------------

clear
%rng('default') %seed random variables
global c_0;
c_0 = 299792458;

Pedestrian = 0;
Bicycle = 0;
Car = 0;
synteticTarget = 1; %use Signal simulation for synt point targets in simulateSignal.m

%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();

%% Pedestrian Target
if Pedestrian
    ped = backscatterPedestrian;
    %ped.Name = 'Pedestrian1';
    ped.Height = 1+rand(); % [1m,2m]
    ped.WalkingSpeed = 1.4*ped.Height;
%     ped.WalkingSpeed = 1; %in m/s
    ped.OperatingFrequency = fmcw.f0;
    ped.PropagationSpeed = c_0; %propagation speed of radar rays in air
    posx = 25*rand();
    posy = 25*rand();
    ped.InitialPosition = [4; 0; 0]; %add random posx posy
    ped.InitialHeading = 0; %in degree, heading along x from x=5 to x=7
    
    %Ground Truth
    %targetR Range (dist. from radar to target)
    %targetV radial Velocity
    %targetV<0: approaching,    targetV>0: moving away from radar
    targetR = sqrt(ped.InitialPosition(1)^2+ped.InitialPosition(2)^2);
    targetV = +ped.WalkingSpeed*cos(ped.InitialHeading/360*2*pi);
    fprintf('Simulation of Pedestrian starting in Range %d m and radial Velocity %d m/s.\n', ...
                targetR, targetV)
    
    
    %Model Radar Signal for selected Target
    psb = modelSignal(ped, fmcw);
    pRD = fmcw.RDmap(psb);
    fmcw.plotRDmap(pRD, [targetR, targetV]);
end


%% Bycicle Traget
if Bicycle
    bike = backscatterBicyclist;
    bike.NumWheelSpokes = 20;
    bike.GearTransmissionRatio = 1.5; %Ratio of wheel rotations to pedal rotations
    bike.OperatingFrequency = fmcw.f0;
    posx = 25*rand();
    posy = 25*rand();
    bike.InitialPosition = [5;0;0];
    bike.InitialHeading = 0; %in degree, heading along x-axis
    bike.Speed = 3; %m/s
    bike.Coast = false; %Padeling movements?
    bike.PropagationSpeed = c_0; %propagation speed of radar rays in air
    % bike.AzimutAngles = fmcw.azimut; %default 77GHz cyclist <- use default
    % bike.ElevationAngles = fmcw.elevation; %default 77GHz cyclist <- use default
    % bike.RCSPattern = fmcw.RCS; %default 77GHz cyclist <- use default
    
    %Ground Truth
    %targetR Range (dist. from radar to target)
    %targetV radial Velocity
    %targetV<0: approaching,    targetV>0: moving away from radar
    targetR = sqrt(bike.InitialPosition(1)^2+bike.InitialPosition(2)^2);
    targetV = +bike.Speed*cos(bike.InitialHeading/360*2*pi);
    fprintf('Simulation of Bycicle starting in Range %d m and radial Velocity %d m/s.\n', ...
                targetR, targetV)
    
            
    %Model Radar Signal for selected Target
    bsb = modelSignal(bike, fmcw);
    bRD = fmcw.RDmap(bsb);
    fmcw.plotRDmap(bRD, [targetR, targetV]);
end

if Car
    targetsRange = [10];
    targetsVelocity = [5];
    
    csb = simulateSignal(fmcw, targetsRange, targetsVelocity, 0, false);
    cRD = fmcw.RDmap(csb);
    fmcw.plotRDmap(cRD, [targetsRange, targetsVelocity]);
    
end

if synteticTarget
    %Simulate syntetic point targets
    %Enter Positions/Velocities for point targets
    targetsRange = [10];
    targetsVelocity = [5];
    
    csb = simulateSignal(fmcw, targetsRange, targetsVelocity, 0, false);
    cRD = fmcw.RDmap(csb);
    fmcw.plotRDmap(cRD, [targetsRange, targetsVelocity]);
    
end








