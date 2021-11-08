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
pkg load statistics 
pkg load signal

close all
clear all

%% Target List Simulation
tic
global c_0;
c_0 = 299792458;

plotAntennas = [0]; % List indices of RX antenna elements to be plotted in RD map [0: plot RD map, (1:8): Plot each antenna element, []: No Plot]
Szenarios = 1; % SET NUMBER OF SZENARIOS
duration = 0.017; % (sec) SET DURATION OF A SZENARIO  1 meas == 256*64µs = 0.0164 s

%% Setup directories to save results
SimDataPath = 'SimulationData/';
add_files = true;
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
fmcw = fmcw.init_RDmap(); %Initialize FMCW radar object and environment
fmcw = fmcw.generateChirpSequence(); %Generate chirp waveform, initialize fmcw.chirps
fmcw = fmcw.generateAntPattern(); %Generate antenna Pattern, initialize fmcw.antPattern
%fmcw = fmcw.setupMeasurement(); %setup all measurement environment objects for 'modelSignal.m'
tstep = fmcw.chirpsCycle*fmcw.chirpInterval; %duration of one radar measurement cycle

for meas = 1:Szenarios
    status = mkdir([SimDataPath, 'Szenario', num2str(meas+ file_offset)]);
    fprintf('Simulate Szenario %i/%i...\n', meas, Szenarios)
    %% Targets in this Szenario
    % Select random number of targets in this szenario
    checksum = 0;
    while checksum == 0
        Pedestrians = floor(2.3*rand());
        Bicycles = floor(2.3*rand());
        Cars = floor(2.3*rand());
        checksum = Pedestrians+Bicycles+Cars;
    end
    Pedestrians = 0;
    Bicycles = 0;
    Cars = 1;
    
    % OUTPUT: Labels.Target = [Range, Vel, Azi, Heading]
    %Labels = {}; %Collect all Labels in struct
    Targets = cell(Pedestrians+Bicycles+Cars,2); %Collect all Targets in struct
    Tidx = 0;
    %% Pedestrian Target
    for target = 1:Pedestrians
        Tidx = Tidx+1;
        ped = Pedestrian;
        ped.Height = 1.3+rand(); % [1.0m, 2.3m]
        ped.WalkingSpeed = 1+0.7*rand(); %rand()* 1.4*ped.Height 
        randrange = rand()*0.9*fmcw.rangeBins(end);
        randazi = (rand()-0.5)*90;
        randposx = cosd(randazi)*randrange;
        randposy = sind(randazi)*randrange;
        ped.xPos = randposx; %add random posx posy
        ped.yPos = randposy;
        heading = rand()*360-180;
        ped.heading = heading; %in degree, heading along x from x=5 to x=7
        ped = ped.initPedestrian();

        %Ground Truth
        %targetR: Range (radial dist. from radar to target)
        %targetV: radial Velocity <0 approaching, targetV>0 moving away from radar
        targetR = sqrt(ped.xPos^2+ped.yPos^2);
        targetV = +ped.WalkingSpeed*cos(ped.heading/360*2*pi);
        azi = atand(ped.yPos/ped.xPos);

        %Label output and save
        %label = [targetR, targetV, azi, fmcw.egoMotion, ped.InitialPosition(1), ped.InitialPosition(2), 0.65, 0.5, heading, 0];
        name = ['Pedestrian', num2str(target)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = ped;
    end



    %% Bycicle Traget
    for target = 1:Bicycles
        Tidx = Tidx+1;
        bike = Bicyclist;
        bike = bike.initBicycle(floor(1.999* rand()));
        randrange = rand()*0.9*fmcw.rangeBins(end);
        randazi = (rand()-0.5)*90;
        randposx = cosd(randazi)*randrange;
        randposy = sind(randazi)*randrange;
        randvel = 1.5+8*rand(); %10m/s top speed
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

        rPos = [0;0;fmcw.height];
        bike = bike.generateBackscatterTarget(fmcw, rPos); %Generate backscattering points with RCS

        %Label output and save
        %label =  [targetR, targetV, azi, fmcw.egoMotion, bike.xPos, bike.yPos, bike.width, bike.length, heading, 0];
        name = ['Bicycle', num2str(bike.typeNr)];
        Targets{Tidx,1} = name;
        Targets{Tidx,2} = bike;
    end

    %% Car target
    for target = 1:Cars
        Tidx = Tidx+1;
        car = Car;
        car = car.initCar(0); %floor(1.999* rand())
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

        rPos = [0;0;fmcw.height];
        car = car.generateBackscatterTarget(fmcw,rPos); %Generate backscattering points with RCS

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
    
    % ParFor each measurement step in this scenario TIME: duration [sec]
    parfor tidx = 1:floor(duration/tstep)
        %% Move Targets
        [MovedTargets, Label, rPlt] = move_TrajectoryPlanner(Traj, tidx, Targets, egoMotion);
        rPos = rPlt(:,1); %current radar position
        
        sz = size(MovedTargets);        
        for i = 1:sz(1)
            MovedTargets{i,2} = MovedTargets{i,2}.generateBackscatterTarget(fmcw,rPos); %update reflection point positions
        end
        
        %% Generate obstruction map for each chirp/time step
        map = generateObstructionMap(MovedTargets, fmcw, rPos);


        %% Simulate Baseband Signals
        %for each target in Targets
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
            [sbTarget, obstruction] = modelBasebandSignal(Target, targetID, map, rPlt, fmcw);
            
            %Update Label
            Label{i,2}(end) = obstruction; %set obstruction factor in Label (1:visible, ..., 4: fully hidden)
            if rPos(1)>0 || rPos(2)>0
                % re-calculate Label relative to current radar position
                tx = Label{i,2}(5);
                ty = Label{i,2}(6);
                heading = Label{i,2}(9);
                Label{i,2}(1) = sqrt((tx-rPos(1))^2+(ty-rPos(2))^2); % R
                Label{i,2}(3) = atand((ty-rPos(2))/(tx-rPos(1))); %azi
                if strcmp(MovedTargets{i,1}(1:3), 'Ped')
                    Label{i,2}(2) = cosd(Label{i,2}(3)-heading)*Target.WalkingSpeed; % vD
                else
                    Label{i,2}(2) = cosd(Label{i,2}(3)-heading)*Target.vel; % vD
                end
                Label{i,2}(5) = tx-rPos(1); % xPos
                Label{i,2}(6) = ty-rPos(2); % yPos
            end
            
            % Power correction for Bikes/Vehicles
            sbTarget = sbTarget/2;        
            
            sb =  sb + sbTarget;
        end

        % Add Noise and static Clutter to baseband signal
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
toc