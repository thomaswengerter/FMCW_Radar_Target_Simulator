function [sb] = modelSignal(target, fmcw)
%% modelSignal() Summary
%   This function takes the configured radar 'target' object (from phased
%   toolbox) as input and generates the radar response for the radar
%   settings specified in FMCWradar Object 'radar'


% Add random static clutter Targets
maxClutterObjects = 30;
numClutterObjects = ceil(rand()*maxClutterObjects);


%% Start Radar Measurement
% Collect sampled reflections within the current chirpInterval for L consecutive chirps
% Sampling frequency for propagation is Propagation_fs = sweepBw to avoid
% undersampling
xRX = complex(zeros(round(fmcw.chirpInterval*fmcw.Propagation_fs),fmcw.L, fmcw.RXant));

if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    tsamp = fmcw.chirpInterval; % timestep to move target & radar
    xTX = fmcw.MStrx(fmcw.chirps()); % Radar transmitter signal
    
    %Target Backscatter
    if ~isempty(target)
        for chirp = 1:fmcw.L
            % Looping through chirps
            [posr,velr,axr] = fmcw.MSradarplt(tsamp); % current Position of Radar
            %tseq = fmcw.chirpsCycle*fmcw.chirpInterval; % Duration of radar measurement
            [post,velt,axt] = move(target,tsamp,target.InitialHeading); % start Moving target
            [~,angle] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
            shape = size(post);
            N = shape(end); % getNumScatters(target)
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
            
            CarTarget = 0;
            try
                % TODO: Find better Criteria
                CarTarget = strcmp(target.ID(1:end-1), 'Vehicle');
                if CarTarget == 0
                    error('Could not identify Car ID %s!', car.ID)
                end
            catch
                %No car
            end
                  
            % TODO: Check why mangle required for other targets????
            if CarTarget
                angle(angle>180) = angle(angle>180)-360;
                xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, angle, fmcw.f0, fmcw.c0)); 
            else
                xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, mangle, fmcw.f0, fmcw.c0)); 
            end
        end
    end

    
    %NOT RECOMMENDED: Backscatter Static Clutter
    % Rather use function fmcw.addStaticClutter(s_beat) for better
    % simulation performance
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
                [posr,velr,axr] = fmcw.MSradarplt(tsamp);
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
    
    % Radar RX Signal Processing
    xRX = xRX + cRX; %add target and clutter signals
    sb = zeros(size(xRX));
    for ant = 1:fmcw.RXant
        sb(:,:,ant) = conj(dechirp(xRX(:,:,ant), xTX)); %Mix Tx with Rx to transform sRx to baseband signal       
    end
    sb = real(sb);
    
    % AD Downsampling fs
    idxstep = fmcw.Propagation_fs/fmcw.fs;
    sb = sb(1:idxstep:end,:,:);
    
    % Crop TX pause between chirps
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

