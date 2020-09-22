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
plotAntennas = []; %list indices of RX antenna elements to be plotted in RD map

% Select number of target samples
Pedestrians = 1;
Bicycles = 0;
Cars = 0;
NoTarget = 0; 
Syntetics = 0; %use Signal simulation for synt point targets in simulateSignal.m

%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();
% Initialize FMCW radar object and environment
fmcw = fmcw.generateChirpSequence(); %Generate chirp waveform, initialize fmcw.chirps
fmcw = fmcw.generateAntPattern(); %Generate antenna Pattern, initialize fmcw.antPattern
fmcw = fmcw.setupMeasurement(); %setup all measurement environment objects for 'modelSignal.m'


% Set dictionary to save results
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

fprintf('Simulate Pedestrians...\n ')
parfor target = 1:Pedestrians
    ped = backscatterPedestrian;
    ped.Height = 1+rand()*1.3; % [1.0m, 2.3m]
    ped.WalkingSpeed = rand()* 1.4*ped.Height; % 
    ped.OperatingFrequency = fmcw.f0;
    ped.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
    randrange = rand()*0.9*fmcw.rangeBins(end);
    randazi = (rand()-0.5)*90;
    randposx = cosd(randazi)*randrange;
    randposy = sind(randazi)*randrange;
    ped.InitialPosition = [randposx; randposy; 0]; %add random posx posy
    %ped.InitialPosition = [5; 0; 0];
    heading = rand()*360-180;
    %heading = 0;
    ped.InitialHeading = heading; %in degree, heading along x from x=5 to x=7

    %Ground Truth
    %targetR: Range (radial dist. from radar to target)
    %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
    targetR = sqrt(ped.InitialPosition(1)^2+ped.InitialPosition(2)^2);
    targetV = +ped.WalkingSpeed*cos(ped.InitialHeading/360*2*pi);
    azi = atand(ped.InitialPosition(2)/ped.InitialPosition(1));
    
    %Model Radar Signal for selected Target
    [sb,~] = modelSignal(ped, 1, [], fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    pRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(pRD, [], plotAntennas);
    %plotNoise; 
    
    %Label output and save
    label = [targetR, targetV, azi, fmcw.egoMotion, ped.InitialPosition(1), ped.InitialPosition(2), 0.65, 0.5, heading];
    saveMat(pRD, label, 'Pedestrian', target+file_offset, SimDataPath)
end
fprintf('Done!\n')


%% Bycicle Traget
if Bicycles, status = mkdir([SimDataPath,'Bicycle']); end%create dict
file_offset = 0; %offset to keep existing files
if Bicycles && add_files
    files = dir([SimDataPath,'Bicycle/Bicycle*']);
    file_offset = length(files); % #files to keep  
end

fprintf('Simulate Bicycles...\n')
parfor target = 1:Bicycles
    bike = Bicyclist;
    bike = bike.initBicycle(floor(1.999* rand()));
    randrange = rand()*0.9*fmcw.rangeBins(end);
    randazi = (rand()-0.5)*90;
    randposx = cosd(randazi)*randrange;
    randposy = sind(randazi)*randrange;
    randvel = rand()*10; %10m/s top speed
    heading = rand()*360-180;
    bike.xPos = randposx; % x dist from radar
    bike.yPos = randposy; % y dist from radar
    bike.vel = randvel; %m/s
    bike.heading = heading; %degrees, from x-axis

    % MATLAB phased Toolbox Bicyclist
%     bike = backscatterBicyclist;
%     Spokes = [20, 24, 28, 32, 36];
%     bike.NumWheelSpokes = Spokes(ceil(rand()*length(Spokes)));
%     bike.GearTransmissionRatio = 1.5; %Ratio of wheel rotations to pedal rotations
%     bike.OperatingFrequency = fmcw.f0;
%     randposx = fmcw.rangeBins(end)*rand();
%     randposy = randposx* (rand()-0.5);
%     bike.InitialPosition = [randposx;randposy;0];
%     heading = rand()*360-180;
%     bike.InitialHeading = heading; %in degree, heading along x-axis
%     randspeed = rand()*10; %max 10m/s
%     bike.Speed = randspeed; %m/s
%     bike.Coast = false; %Padeling movements?
%     bike.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
%     % bike.AzimutAngles = fmcw.azimut; %default 77GHz cyclist <- use default
%     % bike.ElevationAngles = fmcw.elevation; %default 77GHz cyclist <- use default
%     % bike.RCSPattern = fmcw.RCS; %default 77GHz cyclist <- use default
%     
    %Ground Truth
    %targetR: Range (radial dist. from radar to target)
    %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
    targetR = sqrt(bike.InitialPosition(1)^2+bike.InitialPosition(2)^2);
    targetV = +bike.Speed*cos(bike.InitialHeading/360*2*pi);
    azi = atand(bike.InitialPosition(2)/bike.InitialPosition(1));
            
    %Model Radar Signal for selected Target
    [sb,~] = modelSignal(bike, 2, [],fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    bRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(bRD, [], plotAntennas);
    %plotNoise;

    %Label output and save
    label = [targetR, targetV, azi, fmcw.egoMotion, bike.InitialPosition(1), bike.InitialPosition(2), 0.65, 2, heading];
    saveMat(bRD, label, 'Bicycle', target+file_offset, SimDataPath)
end
fprintf('Done!\n')


%% Car target
if Cars, status = mkdir([SimDataPath,'Car']); end %create dict
file_offset = 0; %offset to keep existing files
if Cars && add_files
    files = dir([SimDataPath,'Car/Car*']);
    file_offset = length(files); % #files to keep  
end

fprintf('Simulate Cars...')
parfor target = 1:Cars
    car = Car;
    car = car.initCar(floor(1.999* rand()));
    randrange = rand()*0.9*fmcw.rangeBins(end);
    randazi = (rand()-0.5)*90;
    randposx = cosd(randazi)*randrange;
    randposy = sind(randazi)*randrange;
    randvel = rand() * fmcw.velBins(end);
    heading = rand()*360-180;
    car.xPos = randposx; % x dist from radar
    car.yPos = randposy; % y dist from radar
    car.vel = randvel; %m/s
    car.heading = heading; %degrees, from x-axis
    
    
    % Calculate label
    relangle = atand(car.yPos/car.xPos)-car.heading; %angle between heading and radial velocity
    targetR = sqrt(car.xPos^2+car.yPos^2); %radial distance
    targetV = cosd(relangle)*car.vel; %radial velocity
    azi = atand(car.yPos/car.xPos);
    
    car = car.generateBackscatterTarget(fmcw); %Generate backscattering points with RCS
    
    
    [sb,~] = modelSignal(car, 3+car.typeNr, [], fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    cRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(cRD, [], plotAntennas);
    %plotNoise;
    

    %Label output and save
    label = [targetR, targetV, azi, fmcw.egoMotion, car.xPos, car.yPos, car.width, car.length, heading, 0];
    saveMat(cRD, label, 'Car', target+file_offset, SimDataPath)
end
fprintf('Done!\n')


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
    fmcw.plotRDmap(sRD, [], plotAntennas);
    
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

fprintf('Simulate empty spectra...')
parfor target = 1:NoTarget
    %Simulate Noise
    [sb,~] = modelSignal([], [], [], fmcw);
    sbn = fmcw.addGaussNoise(sb);
    sbc = fmcw.addStaticClutter(sbn);
    nRD = fmcw.RDmap(sbc);
    fmcw.plotRDmap(nRD, [], plotAntennas);
    
    %DEBUG: Show Noise char over Range
    %plotNoise;
    
    %Label output and save
    label = [[],[],[],[],[],[],[],[],[],[]];
    saveMat(nRD, label, 'NoTarget', target+file_offset, SimDataPath)
end
fprintf('Done!\n')

toc



