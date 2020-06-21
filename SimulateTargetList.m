%% SIMULATE TARGET LIST
% Function to start simulation for a list of targets generated from a time
% sequence in a specific scenario.
% 
% All targets are initialized randomly and collected in one struct Target.
% Also the labels are stored in struct Label.
% When all targets are positioned, the obstruction map is generated.
% Finally, signals are generated in modelSignal for each target considering
% the obstruction map.
% 
% -------------------------------------------------------------------------
close all
clear
%% Target Simulation
global c_0;
c_0 = 299792458;
plotAntennas = []; %list indices of RX antenna elements to be plotted in RD map
Szenarios = 1; % SET NUMBER OF SZENARIOS
duration = 3; % SET DURATION OF A SZENARIO (sec)

%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();
% Initialize FMCW radar object and environment
fmcw = fmcw.generateChirpSequence(); %Generate chirp waveform, initialize fmcw.chirps
fmcw = fmcw.generateAntPattern(); %Generate antenna Pattern, initialize fmcw.antPattern
fmcw = fmcw.setupMeasurement(); %setup all measurement environment objects for 'modelSignal.m'
tstep = fmcw.chirpsCycle*fmcw.chirpInterval;

% Set dictionary to save results
SimDataPath = 'SimulationData/';
add_files = false;
if ~add_files && exist(SimDataPath(1:end-1),'dir')
    %clear Sim data folder
    rmdir('SimulationData','s')
end

file_offset = 0; %offset to keep existing files
if add_files
    %Check for existing simulation data files
    files = dir([SimDataPath,'/Szenario*']);
    file_offset = length(files); % #files to keep
end


for meas = 1:Szenarios
    status = mkdir([SimDataPath, 'Szenario', num2str(meas+ file_offset)]);
    fprintf('Simulate Szenario %i/%i...\n', meas, Szenarios)
    %% Targets in this Szenario
    % Select random number of targets in this szenario
    Pedestrians = floor(2.5*rand());
    %Pedestrians = 1;
    Bicycles = floor(1.5*rand());
    %Bicycles = 0;
    Cars = floor(3*rand());
    %Cars = 0;
    
    % OUTPUT: Labels.Target = [Range, Vel, Azi, Heading]
    %Labels = {}; %Collect all Labels in struct
    Targets = cell(Pedestrians+Bicycles+Cars,2); %Collect all Targets in struct
    Tidx = 0;
    %% Pedestrian Target
    for target = 1:Pedestrians
        Tidx = Tidx+1;
        ped = backscatterPedestrian;
        ped.Height = 1+rand()*1.3; % [1.0m, 2.3m]
        ped.WalkingSpeed = rand()* 1.4*ped.Height; %  
        ped.OperatingFrequency = fmcw.f0;
        ped.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
        randposx = fmcw.rangeBins(end)*rand();
        randposy = randposx* 2* (rand()-0.5);
        ped.InitialPosition = [randposx; randposy; 0]; %add random posx posy
        heading = rand()*360-180;
        ped.InitialHeading = heading; %in degree, heading along x from x=5 to x=7

        %Ground Truth
        %targetR: Range (radial dist. from radar to target)
        %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
        targetR = sqrt(ped.InitialPosition(1)^2+ped.InitialPosition(2)^2);
        targetV = +ped.WalkingSpeed*cos(ped.InitialHeading/360*2*pi);
        azi = atand(ped.InitialPosition(2)/ped.InitialPosition(1));

        %Label output and save
        %label = [targetR, targetV, azi, fmcw.egoMotion, ped.InitialPosition(1), ped.InitialPosition(2), 0.65, 0.5, heading, 0];
        name = ['Pedestrian', num2str(target)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = ped;
    end



    %% Bycicle Traget
    for target = 1:Bicycles
        Tidx = Tidx+1;
        bike = backscatterBicyclist;
        Spokes = [20, 24, 28, 32, 36];
        bike.NumWheelSpokes = Spokes(ceil(rand()*length(Spokes)));
        bike.GearTransmissionRatio = 1.5; %Ratio of wheel rotations to pedal rotations
        bike.OperatingFrequency = fmcw.f0;
        randposx = fmcw.rangeBins(end)*rand();
        randposy = randposx* 2*(rand()-0.5);
        bike.InitialPosition = [randposx;randposy;0];
        heading = rand()*360-180;
        bike.InitialHeading = heading; %in degree, heading along x-axis
        randspeed = rand()*10; %max 10m/s
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
        azi = atand(bike.InitialPosition(2)/bike.InitialPosition(1));

        %Label output and save
        %label = [targetR, targetV, azi, fmcw.egoMotion, bike.InitialPosition(1), bike.InitialPosition(2), 0.65, 2, heading, 0];
        name = ['Bicycle', num2str(target)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = bike;
    end


    %% Car target
    for target = 1:Cars
        Tidx = Tidx+1;
        car = Car;
        car = car.initCar(0);
        randxpos = rand()*fmcw.rangeBins(end);
        randypos = randxpos*2*(rand()-0.5);
        randvel = rand() * fmcw.velBins(end);
        heading = rand()*360-180;
        car.xPos = randxpos; % x dist from radar
        car.yPos = randypos; % y dist from radar
        car.vel = randvel; %m/s
        car.heading = heading; %degrees, from x-axis


        % Calculate label
        relangle = atand(car.yPos/car.xPos)-car.heading; %angle between heading and radial velocity
        targetR = sqrt(car.xPos^2+car.yPos^2); %radial distance
        targetV = cosd(relangle)*car.vel; %radial velocity
        azi = atand(car.yPos/car.xPos);

        car = car.generateBackscatterTarget(fmcw); %Generate backscattering points with RCS

        %Label output and save
        %label =  [targetR, targetV, azi, fmcw.egoMotion, car.xPos, car.yPos, car.width, car.length, heading, 0];
        name = ['Vehicle', num2str(car.typeNr)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = car;
    end
    
    %% Plan Trajectory of targets
    Traj = TrajectoryPlanner;
    Traj = init_TrajectoryPlanner(Traj, tstep, duration, Targets, fmcw);
    egoMotion = fmcw.egoMotion;
    timesteps = 0:tstep:duration;
    % For each measurement step in this scenario TIME: 10 SECONDS
    parfor tidx = 1:floor(duration/tstep)
        %% Move Targets
        t = timesteps(tidx);
        [MovedTargets, Label] = move_TrajectoryPlanner(Traj, tidx, Targets, egoMotion);
        
        %% Generate obstruction map for each chirp/time step
        map = generateObstructionMap(MovedTargets, fmcw);


        %% Simulate Baseband Signals
        %for each target in Targets
        sb = zeros(fmcw.K, fmcw.L, fmcw.RXant); %empty Rx signal
        sz = size(MovedTargets);
        for i= 1:sz(1)
            Target = MovedTargets{i,2};
            if strcmp(MovedTargets{i,1}(1:3), 'Ped')
                targetID = 1;
            elseif strcmp(MovedTargets{i,1}(1:4),'Bicy')
                targetID = 2;
            elseif strcmp(MovedTargets{i,1}(1:4), 'Vehi')
                targetID = 3+Target.typeNr;
            end
            %Model Radar Signal for selected Target
            [sbTarget, obstruction] = modelSignal(Target, targetID, map, fmcw);
            Label{i,2}(end) = obstruction; %set obstruction factor in Label (1:visible, ..., 4: fully hidden)
            
            sb =  sb + sbTarget;
        end

        % Add Noise and static Clutter to baseband signal
        sbn = fmcw.addGaussNoise(sb);
        sbc = fmcw.addStaticClutter(sbn);
        RD = fmcw.RDmap(sbc);
        fmcw.plotRDmap(RD, [], plotAntennas);

        %DEBUG: Show Noise char over Range
        %plotNoise;

        saveMat(RD, Label, ['Szenario', num2str(meas+file_offset)], round(t/tstep), SimDataPath)
    end
end
fprintf('Simulation completed.\n')

%% Generate Sample Target List
% Fields in Target struct:
% CarXX:        [xPos, yPos, vel, heading, type]
% PedestrianXX: [xPos, yPos, vel, heading]
% BicycleXX:    [xPos, yPos, vel, heading]

% TargetProperties = {};
% TargetNames = {};

% TargetNames{1} = 'Car';
% TargetProperties{1} = [10, 3, 8, 160, 0];
% TargetNames{2} = 'Pedestrian';
% TargetProperties{2} = [3, -3, 2, 10];

% Targets = cell2struct(TargetProperties, TargetNames, 2);
% NoTargets = numel(fieldnames(Targets));


% TargetLists = {};
% % TargetList{MeasNr:, TargetType, TargetValues}
% TargetLists{1,1,1} = 'Car';
% TargetLists{1,1,2} = [10, 3, 8, 160, 0];
% TargetLists{1,2,1} = 'Pedestrian';
% TargetLists{1,2,2} = [3, -3, 2, 10];
% 
% TargetLists{2,1,1} = 'Car';
% TargetLists{2,1,2} = [10, 3, 8, 160, 0];
% TargetLists{2,2,1} = 'Pedestrian';
% TargetLists{2,2,2} = [3, -3, 2, 10];
% 
% TargetLists{3,1,1} = 'Car';
% TargetLists{3,1,2} = [10, 3, 8, 160, 0];

