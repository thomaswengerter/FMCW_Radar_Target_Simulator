%% SIMULATE TARGET LIST
% Function to start simulation for a list of targets generated from a time
% sequence of measurements in a road scenario.
% 
% I. All targets are initialized randomly and collected in one struct
% 'Target'. Also the labels are stored in struct 'Label'.
% 
% II. For the 'duration' of the measurement, realistic trajectories for 
% each individual target are planned in the 'TrajectoryPlanner'. 
% 
% III. In each measurement step, the current positions of the targets are 
% read from the 'TrajectoryPlanner' object. When all targets are postioned 
% accordingly, the 'obstructionMap' is generated for this measurement step.
% 
% IV. Finally, reflection signals are generated in 'modelSignal' for each 
% target considering the 'obstructionMap'.  
% 
% V. Simulated Noise and Clutter are added. The RD map is calculated by
% FFTs with Hanning Windowing in the 'RDmap()' function. Save Labels and
% RD map in mat-file.
% 
% 
% OUTPUTS:
% Label Format:
% {TargetID;  [targetR, targetV, azi, egoMotion, xPos, yPos, width, length,
%               heading, obstruction]}
% RD map:
% [160, 256, 16] -> (Rangebins, Velbins, RXant*2)
% RD spectrum in dB
% 
% Coordinate Systems:
% (x,y) -> x: Radar's line of sight   -> y: 90° to the left from radars LoS
% -------------------------------------------------------------------------

close all
clear

%% Target List Simulation
global c_0;
c_0 = 299792458;

plotAntennas = []; %list indices of RX antenna elements to be plotted in RD map
Szenarios = 100; % SET NUMBER OF SZENARIOS
duration = 1; % (sec) SET DURATION OF A SZENARIO  1 meas == 256*64µs = 0.0164 s

%% Setup directories to save results
SimDataPath = 'SimulationData/';
add_files = false;
if ~add_files && exist(SimDataPath(1:end-1),'dir')
    %clear Sim data folder
    rmdir('SimulationData','s');
    if exist('YOLOpics','dir')
        rmdir('YOLOpics','s');
    end
end

file_offset = 0; %offset to keep existing files
if add_files && file_offset == 0
    %Check for existing simulation data files
    files = dir([SimDataPath,'/Szenario*']);
    file_offset = length(files); % #files to keep
end

%% Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();
% Initialize FMCW radar object and environment
fmcw = fmcw.generateChirpSequence(); %Generate chirp waveform, initialize fmcw.chirps
fmcw = fmcw.generateAntPattern(); %Generate antenna Pattern, initialize fmcw.antPattern
fmcw = fmcw.setupMeasurement(); %setup all measurement environment objects for 'modelSignal.m'
tstep = fmcw.chirpsCycle*fmcw.chirpInterval; %duration of one radar measurement cycle

for meas = 1:Szenarios
    status = mkdir([SimDataPath, 'Szenario', num2str(meas+ file_offset)]);
    fprintf('Simulate Szenario %i/%i...\n', meas, Szenarios)
    %% I. Initialize Targets in this Szenario
    % Select random number of targets in this szenario
    checksum = 0;
    while checksum == 0
        Pedestrians = floor(2.3*rand());
        Bicycles = floor(2.3*rand());
        Cars = floor(2.3*rand());
        checksum = Pedestrians+Bicycles+Cars;
    end
    Targets = cell(Pedestrians+Bicycles+Cars,2); %Collect all Targets in struct
    Tidx = 0;
    
    
    %% I.1 Pedestrian Target
    for target = 1:Pedestrians
        Tidx = Tidx+1;
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
        ped.InitialHeading = heading; %in degree

        %Ground Truth
        %targetR: Range (radial dist. from radar to target)
        %targetV: radial Velocity <0 approaching, >0 moving away from radar
        targetR = sqrt(ped.InitialPosition(1)^2+ped.InitialPosition(2)^2);
        targetV = +ped.WalkingSpeed*cos(ped.InitialHeading/360*2*pi);
        azi = atand(ped.InitialPosition(2)/ped.InitialPosition(1));

        %Label output and save
        %label = [targetR, targetV, azi, fmcw.egoMotion, ped.InitialPosition(1), ped.InitialPosition(2), 0.65, 0.5, heading, 0];
        name = ['Pedestrian', num2str(target)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = ped;
    end


    %% I.2 Bycicle Traget
    for target = 1:Bicycles
        Tidx = Tidx+1;
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


        % Calculate label
        relangle = atand(bike.yPos/bike.xPos)-bike.heading; %angle between heading and radial velocity
        targetR = sqrt(bike.xPos^2+bike.yPos^2); %radial distance
        targetV = cosd(relangle)*bike.vel; %radial velocity
        azi = atand(bike.yPos/bike.xPos);

        bike = bike.generateBackscatterTarget(fmcw); %Generate backscattering points with RCS

        %Label output and save
        %label =  [targetR, targetV, azi, fmcw.egoMotion, bike.xPos, bike.yPos, bike.width, bike.length, heading, 0];
        name = ['Bicycle', num2str(bike.typeNr)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = bike;
    end

%% Bicycle Target with MATLAB phased Toolbox
%     for target = 1:Bicycles
%         Tidx = Tidx+1;
%         bike = backscatterBicyclist;
%         Spokes = [20, 24, 28, 32, 36];
%         bike.NumWheelSpokes = Spokes(ceil(rand()*length(Spokes)));
%         bike.GearTransmissionRatio = 1.5; %Ratio of wheel rotations to pedal rotations
%         bike.OperatingFrequency = fmcw.f0;
%         randposx = fmcw.rangeBins(end)*rand();
%         randposy = randposx* 2*(rand()-0.5);
%         bike.InitialPosition = [randposx;randposy;0];
%         bike.InitialPosition = [4;2;0];
%         heading = rand()*360-180;
%         heading = -90;
%         bike.InitialHeading = heading; %in degree, heading along x-axis
%         randspeed = rand()*10; %max 10m/s
%         randspeed = 6;
%         bike.Speed = randspeed; %m/s
%         bike.Coast = false; %Padeling movements?
%         bike.PropagationSpeed = fmcw.c0; %propagation speed of radar rays in air
%         % bike.AzimutAngles = fmcw.azimut; %default 77GHz cyclist <- use default
%         % bike.ElevationAngles = fmcw.elevation; %default 77GHz cyclist <- use default
%         % bike.RCSPattern = fmcw.RCS; %default 77GHz cyclist <- use default
% 
%         %Ground Truth
%         %targetR: Range (radial dist. from radar to target)
%         %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
%         targetR = sqrt(bike.InitialPosition(1)^2+bike.InitialPosition(2)^2);
%         targetV = +bike.Speed*cos(bike.InitialHeading/360*2*pi);
%         azi = atand(bike.InitialPosition(2)/bike.InitialPosition(1));
% 
%         %Label output and save
%         %label = [targetR, targetV, azi, fmcw.egoMotion, bike.InitialPosition(1), bike.InitialPosition(2), 0.65, 2, heading, 0];
%         name = ['Bicycle', num2str(target)];
%         Targets{Tidx,1} = name;
%         Targets{Tidx,2} = bike;
%     end


    %% I.3 Car target
    for target = 1:Cars
        Tidx = Tidx+1;
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

        %Label output and save
        %label =  [targetR, targetV, azi, fmcw.egoMotion, car.xPos, car.yPos, car.width, car.length, heading, 0];
        name = ['Vehicle', num2str(car.typeNr)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = car;
    end
    
    
    
    %% II. Plan Trajectory of targets
    Traj = TrajectoryPlanner;
    Traj = init_TrajectoryPlanner(Traj, tstep, duration, Targets, fmcw);
    egoMotion = fmcw.egoMotion;
    
    
    % ParFor each measurement step in this scenario 
    % total TIME: 'duration', measurement TIME: 'tstep'
    parfor tidx = 1:floor(duration/tstep)
        %% III. Generate obstruction map for each chirp/time step
        % Move Targets
        [MovedTargets, Label] = move_TrajectoryPlanner(Traj, tidx, Targets, egoMotion);
        sz = size(MovedTargets);        
        for i = 1:sz(1)
            if strcmp(MovedTargets{i,1}(1:4), 'Vehi') || strcmp(MovedTargets{i,1}(1:3), 'Bic')
                MovedTargets{i,2} = MovedTargets{i,2}.generateBackscatterTarget(fmcw); %update car reflection point positions
            end
        end
        
        % Generate ObstructionMap
        map = generateObstructionMap(MovedTargets, fmcw);


        %% IV. Simulate Baseband Signals
        %Superpostion of simulated reflection signals in the baseband
        sb = zeros(fmcw.K, fmcw.L, fmcw.RXant); %empty Rx signal
        for i= 1:sz(1)
            Target = MovedTargets{i,2};
            if strcmp(MovedTargets{i,1}(1:3), 'Ped')
                targetID = 1; 
            elseif strcmp(MovedTargets{i,1}(1:4),'Bicy')
                targetID = 2;
            else % strcmp(MovedTargets{i,1}(1:4), 'Vehi')
                targetID = 3;
            end
            %Model Radar Signal for selected Target
            [sbTarget, obstruction] = modelSignal(Target, targetID, map, fmcw);
            Label{i,2}(end) = obstruction; %set obstruction factor in Label (1:visible, ..., 4: fully hidden)
            
            if targetID == 1
                % Power correction for Pedestrian
                sbTarget = sbTarget*3;
            else
                % Power correction for Bikes/Vehicles
                sbTarget = sbTarget/2;  
            end
            
            sb =  sb + sbTarget;
        end

        %% V. Add Noise and static Clutter to baseband signal
        sbn = fmcw.addGaussNoise(sb);
        sbc = fmcw.addStaticClutter(sbn);
        RD = fmcw.RDmap(sbc);
        fmcw.plotRDmap(RD, [], plotAntennas);

        %DEBUG: Show Noise char over Range
        %plotNoise;

        saveMat(RD, Label, ['Szenario', num2str(meas+file_offset)], tidx, SimDataPath)
    end
end

fprintf('Simulation completed.\n')