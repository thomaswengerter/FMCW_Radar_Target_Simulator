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

% Select number of target samples
Pedestrians = 2;
Bicycles = 0;
Cars = 0;
synteticTarget = 0; %use Signal simulation for synt point targets in simulateSignal.m


%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();


%% Pedestrian Target
parfor target = 1:Pedestrians
    ped = backscatterPedestrian;
    %ped.Name = 'Pedestrian1';
    ped.Height = 1+rand(); % [1m,2m]
    ped.WalkingSpeed = rand()* 1.4*ped.Height;
    ped.OperatingFrequency = fmcw.f0;
    ped.PropagationSpeed = c_0; %propagation speed of radar rays in air
    randposx = fmcw.rangeBins(end)*rand();
    ped.InitialPosition = [randposx; 0; 0]; %add random posx posy
    randangle = rand()*360;
    ped.InitialHeading = randangle; %in degree, heading along x from x=5 to x=7
    
    %Ground Truth
    %targetR: Range (radial dist. from radar to target)
    %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
    targetR = sqrt(ped.InitialPosition(1)^2+ped.InitialPosition(2)^2);
    targetV = +ped.WalkingSpeed*cos(ped.InitialHeading/360*2*pi);
    
    %Model Radar Signal for selected Target
    psb = modelSignal(ped, fmcw);
    pRD = fmcw.RDmap(psb);
    fmcw.plotRDmap(pRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    status = mkdir('SimulationData/Pedestrian');
    writematrix(pRD, ['SimulationData/Pedestrian/Pedestrian',num2str(target)]);
    writematrix(label, ['SimulationData/Pedestrian/Plabel',num2str(target)]);
end


%% Bycicle Traget
parfor target = 1:Bicycles
    bike = backscatterBicyclist;
    bike.NumWheelSpokes = 20;
    bike.GearTransmissionRatio = 1.5; %Ratio of wheel rotations to pedal rotations
    bike.OperatingFrequency = fmcw.f0;
    randposx = fmcw.rangeBins(end)*rand();
    posy = 25*rand();
    bike.InitialPosition = [randposx;0;0];
    randangle = rand()*360;
    bike.InitialHeading = randangle; %in degree, heading along x-axis
    randspeed = rand()*fmcw.velBins(end);
    bike.Speed = randspeed; %m/s
    bike.Coast = false; %Padeling movements?
    bike.PropagationSpeed = c_0; %propagation speed of radar rays in air
    % bike.AzimutAngles = fmcw.azimut; %default 77GHz cyclist <- use default
    % bike.ElevationAngles = fmcw.elevation; %default 77GHz cyclist <- use default
    % bike.RCSPattern = fmcw.RCS; %default 77GHz cyclist <- use default
    
    %Ground Truth
    %targetR: Range (radial dist. from radar to target)
    %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
    targetR = sqrt(bike.InitialPosition(1)^2+bike.InitialPosition(2)^2);
    targetV = +bike.Speed*cos(bike.InitialHeading/360*2*pi);
            
    %Model Radar Signal for selected Target
    bsb = modelSignal(bike, fmcw);
    bRD = fmcw.RDmap(bsb);
    %fmcw.plotRDmap(bRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    status = mkdir('SimulationData/Bicycle');
    writematrix(bRD, ['SimulationData/Bicycle/Bicycle',num2str(target)]);
    writematrix(label, ['SimulationData/Bicycle/Blabel',num2str(target)]);
end

parfor target = 1:Cars
    targetR = [10]; %add more with ;
    targetV = [5]; %add more with ;
    
    csb = simulateSignal(fmcw, targetR, targetV, 0, false);
    cRD = fmcw.RDmap(csb);
    fmcw.plotRDmap(cRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    status = mkdir('SimulationData/Car');
    writematrix(cRD, ['SimulationData/Car/Car',num2str(target)]);
    writematrix(label, ['SimulationData/Car/Car',num2str(target)]);
end

parfor target = 1:synteticTarget
    %Simulate syntetic point targets
    %Enter Positions/Velocities for point targets
    targetR = [10]; %add more with ;
    targetV = [5]; %add more with ;
    
    csb = simulateSignal(fmcw, targetR, targetV, 0, false);
    cRD = fmcw.RDmap(csb);
    fmcw.plotRDmap(cRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    status = mkdir('SimulationData/Syntetic');
    writematrix(cRD, ['SimulationData/Syntetic/Syntetic',num2str(target)]);
    writematrix(label, ['SimulationData/Syntetic/Slabel',num2str(target)]);
end








