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
Syntetics = 0; %use Signal simulation for synt point targets in simulateSignal.m

% Set dictionary to save files
SimDataPath = 'SimulationData/';
add_files = true;

%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();


%% Pedestrian Target
if Pedestrians, status = mkdir([SimDataPath,'Pedestrian']); end %make dict
file_offset = 0; %offset to keep existing files
if Pedestrians && add_files
    files = dir([SimDataPath,'Pedestrian/Pedestrian*']);
    file_offset = length(files); % #files to keep  
end

parfor target = 1:Pedestrians
    ped = backscatterPedestrian;
    %ped.Name = 'Pedestrian1';
    ped.Height = 1+rand(); % [1m,2m]
    ped.WalkingSpeed = rand()* 1.4*ped.Height;
    ped.OperatingFrequency = fmcw.f0;
    ped.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
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
    %fmcw.plotRDmap(pRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    saveMat(pRD, label, 'Pedestrian', target+file_offset, SimDataPath)
end



%% Bycicle Traget
if Bicycles, status = mkdir([SimDataPath,'Bicycle']); end%create dict
file_offset = 0; %offset to keep existing files
if Bicycles && add_files
    files = dir([SimDataPath,'Bicycle/Bicycle*']);
    file_offset = length(files); % #files to keep  
end

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
    bike.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
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
    saveMat(bRD, label, 'Bicycle', target+file_offset, SimDataPath)
end


%% Car target
if Cars, status = mkdir([SimDataPath,'Car']); end %create dict
file_offset = 0; %offset to keep existing files
if Cars && add_files
    files = dir([SimDataPath,'Car/Car*']);
    file_offset = length(files); % #files to keep  
end
parfor target = 1:Cars
    targetR = [10]; %add more with ;
    targetV = [5]; %add more with ;
    
    csb = simulateSignal(fmcw, targetR, targetV, 0, false);
    cRD = fmcw.RDmap(csb);
    fmcw.plotRDmap(cRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    saveMat(cRD, label, 'Car', target+file_offset, SimDataPath)
end


%% Syntetic targets 
if Syntetics, status = mkdir([SimDataPath,'Syntetic']); end %create dict
file_offset = 0; %offset to keep existing files
if Syntetics && add_files
    files = dir([SimDataPath,'Syntetic/Syntetic*']);
    file_offset = length(files); % #files to keep  
end
parfor target = 1:Syntetics
    %Simulate syntetic point targets
    %Enter Positions/Velocities for point targets
    targetR = [10]; %add more with ;
    targetV = [5]; %add more with ;
    
    csb = simulateSignal(fmcw, targetR, targetV, 0, false);
    sRD = fmcw.RDmap(csb);
    fmcw.plotRDmap(sRD, [targetR, targetV]);
    
    %Label output and save
    label = [targetR, targetV];
    saveMat(sRD, label, 'Syntetic', target+file_offset, SimDataPath)
end




