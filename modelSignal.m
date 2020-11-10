function [sb,obstruction] = modelSignal(target, targetID, map, fmcw)
%% modelSignal() Summary
%   This function takes the configured radar 'target' object (from phased
%   toolbox) as input and generates the radar response for the radar
%   settings specified in FMCWradar Object 'radar'
%
%   INPUTS
%   target:     target object
%   targetID:   1 Pedestrian, 2 Bicycle, >=3 Vehicle
%   map:        Obstruction map for covered target points
%   fmcw:       fmcw radar object

%   OUTPUT
%   sb:         baseband RX signal


%% Start Radar Measurement
% Collect sampled reflections within the current chirpInterval for L consecutive chirps
% Sampling frequency for propagation is Propagation_fs = sweepBw to avoid
% undersampling


%xRX = [fmcw.K, fmcw.L, fmcw.RXant]
xRX = complex(zeros(round(fmcw.chirpInterval*fmcw.Propagation_fs),fmcw.L, fmcw.RXant));

%obstruction factor if target points are covered
obstruction = 0;


if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    tsamp = fmcw.chirpInterval; % timestep to move target & radar
    xTX = fmcw.MStrx(fmcw.chirps()); % Radar transmitter signal
    
    
    %Target Backscatter
    if ~isempty(target)
        for chirp = 1:fmcw.L
            % Looping through chirps
            
            %% Move Targets
            [posr,velr,~] = fmcw.MSradarplt(tsamp); % current Position of Radar
            %tseq = fmcw.chirpsCycle*fmcw.chirpInterval; % Duration of radar measurement
            if targetID >= 3 % Vehicle
                target.CarTarget.release();
                target = restoreReflectionPoints(target, fmcw.c0, fmcw.f0);
                [post,velt,axt,target] = move(target,tsamp,target.InitialHeading); % move car
                %boolidx = round(rand(1,size(post,2))-0.4);
                %target = target.release(boolidx, fmcw.c0, fmcw.f0);
                %post(:,boolidx>0) = [];
                %velt(:,boolidx>0) = [];
                %axt(:,:,boolidx>0) = [];
            elseif targetID == 2 % Bicycle
                target.BicyclistTarget.release();
                target = restoreReflectionPoints(target, fmcw.c0, fmcw.f0);
                [post,velt,axt,target] = move(target,tsamp,target.InitialHeading); % move bike
                %target.release();
                %[post,velt,axt] = move(target,tsamp,target.InitialHeading); % move bike
                % Reduce amount of scatterers randomly
                %boolidx = round(rand(1,size(post,2))+0.4);
                %post(:,boolidx>0) = [];
                %velt(:,boolidx>0) = [];
            else %Pedestrian
                %target.release()
                [post,velt,axt] = move(target,tsamp,target.InitialHeading); % move ped
                %boolidx = round(rand(1,size(post,2)));
                %post(:,boolidx>0) = [];
                %velt(:,boolidx>0) = [];
                %axt(:,:,boolidx>0) = [];
            end
            
            %% Obstruction Map
            if ~isempty(map)
                %Check visibility on map
                ridx = ceil(sqrt(post(1,:).^2+post(2,:).^2)./fmcw.dR);
                azi = round(atand(post(2,:)./post(1,:)));
                targetheight = target.Height; %height of pedestrian/car
                
                CoveredFilter = zeros(size(azi));
                for i = 1:length(azi)
                    if abs(azi(i))<90
                        % map = [tstep, azimuth (-90,+90), Range, (max height obstructed, target ID)]
                        if ridx(i)>size(map,2)
                            % This Scatterer is out of range but will be
                            % considered if not covered
                            if map(azi(i)+90, end, 2)~= targetID && map(azi(i)+90, end, 1)>targetheight
                                %Covered by other Target
                                if map(azi(i)+90, end, 2) == 2
                                    CoveredFilter(i) = 0.5; %Bicyclist only causes partial obstruction
                                else
                                    CoveredFilter(i) = 1; %full obstruction
                                end
                            end
                        elseif map(azi(i)+90, ridx(i), 2)~= targetID && map(azi(i)+90, ridx(i), 1)>targetheight 
                            % This Scatterer is obstructed by other target
                            if map(azi(i)+90, end, 2) == 2
                                CoveredFilter(i) = 0.5; %Bicyclist only causes partial obstruction
                            else
                                CoveredFilter(i) = 1; %full obstruction
                            end
                        elseif map(azi(i)+90, ridx(i), 2)~= targetID && map(azi(i)+90, ridx(i), 1)<=targetheight && map(azi(i)+90, ridx(i), 1)>0
                            % This Scatterer is obstructed by a smaller target
                            if map(azi(i)+90, end, 2) == 2
                                CoveredFilter(i) = 0.5; %Bicyclist only causes partial obstruction
                            else
                                CoveredFilter(i) = 0.5; %half obstruction
                            end
                        end
                    end
                end
                % Partially obstructed Scatterers (behind Bicyclist)
                mask = (CoveredFilter==0.5).* rand(size(CoveredFilter));
                CoveredFilter(CoveredFilter==0.5)= 0;
                CoveredFilter = CoveredFilter + (mask>0.5);
                
                
                
                % Filter covered Scatterers
                if sum(CoveredFilter, 'all')>0
                    obstruction = 1; % some Target points are obstructed
                    if sum(CoveredFilter, 'all') == length(post)
                        obstruction = 4; % all points hidden
                    elseif sum(CoveredFilter, 'all')>(3*length(post)/4)
                        obstruction = 3; % more than 3/4 of the target points are obstructed
                    elseif sum(CoveredFilter, 'all')>(length(post)/2)
                        obstruction = 2; % more than 1/2 of the target points are obstructed
                    end
                    if targetID>=3 % Vehicle
                        post(:,CoveredFilter>=1) = [];
                        velt(:,CoveredFilter>=1) = [];
                        axt(:,:,CoveredFilter>=1) = [];
                        target = RemoveHiddenScatterers(target, CoveredFilter, fmcw.c0, fmcw.f0);
                    elseif targetID == 2 % Bicycle
                        post(:,CoveredFilter>=1) = [];
                        velt(:,CoveredFilter>=1) = [];
                        axt(:,:,CoveredFilter>=1) = [];
                        target = RemoveHiddenScatterers(target, CoveredFilter, fmcw.c0, fmcw.f0);
%                         post(:,CoveredFilter>=1) = [];
%                         velt(:,CoveredFilter>=1) = [];
                    else % Pedestrian
                        % Does not work for backscatterPedestrian Object
                        %post(:,CoveredFilter>=1) = [];
                        %velt(:,CoveredFilter>=1) = [];
                        %axt(:,:,CoveredFilter>=1) = [];
                    end
                                        
                    
                end
            end
            
            %% Reflect Signal from Scatterers
            shape = size(post);
            N = shape(end); % getNumScatters(target)
            if N>0
                [~,angle] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
            
                fmcw.MSchan.release();
                xtrans = fmcw.MSchan(repmat(xTX,1,N),posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
                RXsig = reflect(target,xtrans,angle); %receive sum of Reflections from target

                mangle = mean(angle,2); %angle between target and radar
                if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
                    angle(angle<-100) = angle(angle<-100)+360;
                    mangle = mean(angle,2);
                    if mangle(1)>180
                        mangle(1) = mangle(1)-360;
                    end
                end

                % TODO: Check why mangle required for other targets????
                if targetID >= 2
                    angle(angle>180) = angle(angle>180)-360;
                    xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, angle, fmcw.f0, fmcw.c0)); 
                else
                    xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, mangle, fmcw.f0, fmcw.c0)); 
                end
            end
        end
    end

    
    %% NOT RECOMMENDED: Backscatter Static Clutter Scatterers
    % Rather use function fmcw.addStaticClutter(s_beat) for better
    % simulation performance
    
    % Add random static clutter Targets
    maxClutterObjects = 30;
    numClutterObjects = ceil(rand()*maxClutterObjects);

    cRX = zeros(size(xRX));
    if fmcw.backscatterStatClutter
        for obj = 1:numClutterObjects
            Robj = rand()*fmcw.rangeBins(end)+fmcw.dR; %static Clutter Range
            Azobj = rand()*180-90;
            Elobj = rand()*120-60;
            targetpos = phased.Platform('InitialPosition',[Robj*cos(Azobj)*cos(Elobj); Robj*sin(Azobj)*cos(Elobj); Robj*sin(Elobj)], ...
                    'OrientationAxesOutputPort',true, 'InitialVelocity', [0;0;0], 'Acceleration', [0;0;0]);
            [post,velt,axt] = targetpos(tsamp);

            Rayleigh = raylrnd(1:100);
            Pdistribution = Rayleigh/max(Rayleigh);
            sigma = 1- Pdistribution(ceil(rand()*100)); %RCS
            ClutterObj = phased.RadarTarget('Model','Swerling2','MeanRCS',sigma,...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);

            for chirp = 1:fmcw.L
                [posr,velr,~] = fmcw.MSradarplt(tsamp);
                [~,angle] = rangeangle(posr,post,axt);
                mangle = mean(angle,2); %angle between target and radar
                if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
                    angle(angle<-100) = angle(angle<-100)+360;
                    mangle = mean(angle,2);
                    if mangle(1)>180
                        mangle(1) = mangle(1)-360;
                    end
                end
                fmcw.MSchan.release();
                xtrans = fmcw.MSchan(xTX,posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
                RXclut = ClutterObj(xtrans,true); %receive sum of Reflections from target
                cRX(:,chirp,:) = reshape(cRX(:,chirp,:), size(cRX,1),size(cRX,3)) + fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXclut, mangle, fmcw.f0, fmcw.c0));
            end                       

        end
    end
    
    %% Radar RX Signal Processing
    xRX = xRX + cRX; %add target and clutter signals
    sb = zeros(size(xRX));
    for ant = 1:fmcw.RXant
        sb(:,:,ant) = conj(dechirp(xRX(:,:,ant), xTX)); %Mix Tx with Rx to transform sRx to baseband signal       
    end
    
    % AD Downsampling fs
    idxstep = fmcw.Propagation_fs/fmcw.fs;
    sb = sb(1:idxstep:end,:,:);
    
    sb = real(sb);
    
    %% Crop TX pause between chirps
    if strcmp(fmcw.chirpShape,'SAWgap') && sb(fmcw.K+1,1,1)>0
        % Check for pause after chirp
        error('\nAre you sure you selected the correct Chirp Shape? No gap between SAW chirps detected\n\n')
    else
        % discard the TX pause from the signal
        sb = sb(1:fmcw.K,:,:);
    end
%Received complex signal sb
end
end

