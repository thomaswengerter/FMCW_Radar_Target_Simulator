%-----------------------------------------------------------
%% MAIN: Data Simulation
% Simulate backscattering data of moving radar target objects in 77.6GHz.
% Targets:
% 1. Pedestrians
% 2. Bycicles
% 3. Cars
%-----------------------------------------------------------

clear
tic
%rng('default') %seed random variables
global c_0;
c_0 = 299792458;
plotAntennas = [1]; %list indices of RX antenna elements to be plotted in RD map

% Select number of target samples
Pedestrians = 0;
Bicycles = 0;
Cars = 1;
NoTarget = 0; 
Syntetics = 0; %use Signal simulation for synt point targets in simulateSignal.m

%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();
% Initialize FMCW radar object and environment
fmcw = fmcw.generateChirpSequence(); %Generate chirp waveform, initialize fmcw.chirps
fmcw = fmcw.generateAntPattern(); %Generate antenna Pattern, initialize fmcw.antPattern
fmcw = fmcw.setupMeasurement(); %setup all measurement environment objects for 'modelSignal.m'


% Set dictionary to save files
SimDataPath = 'SimulationData/';
add_files = false;
if ~add_files && exist(SimDataPath(1:end-1),'dir')
    %clear Sim data folder
    rmdir('SimulationData','s')
end


%% Pedestrian Target
if Pedestrians, status = mkdir([SimDataPath,'Pedestrian']); end %make dict
file_offset = 0; %offset to keep existing files
if Pedestrians && add_files
    files = dir([SimDataPath,'Pedestrian/Pedestrian*']);
    file_offset = length(files); % #files to keep  
end

for target = 1:Pedestrians
    ped = backscatterPedestrian;
    ped.Height = 1+rand(); % [1m,2m]
    ped.WalkingSpeed = 1.4*ped.Height; % rand()* 
    ped.OperatingFrequency = fmcw.f0;
    ped.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
    randposx = fmcw.rangeBins(end)*rand();
    randposy = randposx* (rand()-0.5);
    ped.InitialPosition = [randposx; randposy; 0]; %add random posx posy
    heading = rand()*360-180;
    ped.InitialHeading = heading; %in degree, heading along x from x=5 to x=7
    
    %Ground Truth
    %targetR: Range (radial dist. from radar to target)
    %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
    targetR = sqrt(ped.InitialPosition(1)^2+ped.InitialPosition(2)^2);
    targetV = +ped.WalkingSpeed*cos(ped.InitialHeading/360*2*pi);
    
    %Model Radar Signal for selected Target
    sb = modelSignal(ped, fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    pRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(pRD, [targetR,targetV], plotAntennas);
    plotNoise; 
    
    %Label output and save
    label = [targetR, targetV, heading];
    saveMat(pRD, label, 'Pedestrian', target+file_offset, SimDataPath)
end



%% Bycicle Traget
if Bicycles, status = mkdir([SimDataPath,'Bicycle']); end%create dict
file_offset = 0; %offset to keep existing files
if Bicycles && add_files
    files = dir([SimDataPath,'Bicycle/Bicycle*']);
    file_offset = length(files); % #files to keep  
end

for target = 1:Bicycles
    bike = backscatterBicyclist;
    Spokes = [20, 24, 28, 32, 36];
    bike.NumWheelSpokes = Spokes(ceil(rand()*length(Spokes)));
    bike.GearTransmissionRatio = 1.5; %Ratio of wheel rotations to pedal rotations
    bike.OperatingFrequency = fmcw.f0;
    randposx = fmcw.rangeBins(end)*rand();
    randposy = randposx* (rand()-0.5);
    bike.InitialPosition = [randposx;randposy;0];
    heading = rand()*360-180;
    bike.InitialHeading = heading; %in degree, heading along x-axis
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
    sb = modelSignal(bike, fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    bRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(bRD, [], plotAntennas);
    %plotNoise;
    
    %Label output and save
    label = [targetR, targetV, heading];
    saveMat(bRD, label, 'Bicycle', target+file_offset, SimDataPath)
end


%% Car target
if Cars, status = mkdir([SimDataPath,'Car']); end %create dict
file_offset = 0; %offset to keep existing files
if Cars && add_files
    files = dir([SimDataPath,'Car/Car*']);
    file_offset = length(files); % #files to keep  
end
for target = 1:Cars
    car = Car;
    car = car.initCar(0);
    %randxpos = rand()*fmcw.rangeBins(end);
    %randypos = randxpos*(rand()-0.5);
    %heading = rand()*360-180;
    car.xPos = 10; % x dist from radar
    car.yPos = 4; % y dist from radar
    car.heading = 20; %degrees, from x-axis
    car.vel = 5; %m/s
    
    % Calculate label
    relangle = atand(car.yPos/car.xPos)-car.heading; %angle between heading and radial velocity
    targetR = sqrt(car.xPos^2+car.yPos^2); %radial distance
    targetV = cosd(relangle)*car.vel; %radial velocity
    
    cartarget = car.generateBackscatterTarget(fmcw); %Generate backscattering points with RCS
    
    
    sb = modelSignal(cartarget, fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    cRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(cRD, [targetR, targetV], plotAntennas);
    
    %Label output and save
    label = [targetR, targetV, car.heading];
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
    
    sb = simulateSignal(fmcw, targetR, targetV, 0, false);
    sRD = fmcw.RDmap(sb);
    fmcw.plotRDmap(sRD, [targetR, targetV], plotAntennas);
    
    %Label output and save
    label = [targetR, targetV];
    saveMat(sRD, label, 'Syntetic', target+file_offset, SimDataPath)
end


%% Noise / No targets 
if NoTarget, status = mkdir([SimDataPath,'NoTarget']); end %create dict
file_offset = 0; %offset to keep existing files
if NoTarget && add_files
    files = dir([SimDataPath,'NoTarget/NoTarget*']);
    file_offset = length(files); % #files to keep  
end
for target = 1:NoTarget
    %Simulate Noise
    sb = modelSignal([], fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    nRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(nRD, [], plotAntennas);
    
    %DEBUG: Show Noise char over Range
    plotNoise;
    
    %Label output and save
    label = [[],[]];
    saveMat(nRD, label, 'NoTarget', target+file_offset, SimDataPath)
end

toc



