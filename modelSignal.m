function [sb] = modelSignal(target, fmcw)
%% modelSignal() Summary
%   This function takes the configured radar 'target' object (from phased
%   toolbox) as input and generates the radar response for the radar
%   settings specified in FMCWradar Object 'radar'


%% Start Radar Measurement
% Collect sampled reflections within the current chirpInterval for L consecutive chirps
% Sampling frequency for propagation is Propagation_fs = sweepBw to avoid
% undersampling
xRX = complex(zeros(round(fmcw.chirpInterval*fmcw.Propagation_fs),fmcw.L, fmcw.RXant));

if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    tsamp = fmcw.chirpInterval; % timestep to move target & radar
    xTX = fmcw.MStrx(fmcw.chirps()); % Radar transmitter signal
    for chirp = 1:fmcw.L
        % Looping through chirps
        [posr,velr,axr] = fmcw.MSradarplt(tsamp); % current Position of Radar
        %tseq = fmcw.chirpsCycle*fmcw.chirpInterval; % Duration of radar measurement
        [post,velt,axt] = move(target,tsamp,target.InitialHeading); % start Moving target
        [~,angle] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
        shape = size(post);
        N = shape(end); % getNumScatters(target)
        xtrans = fmcw.MSchan(repmat(xTX,1,N),posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
        RXsig = reflect(target,xtrans,angle); %receive sum of Reflections
        mangle = mean(angle,2); %angle between target and radar
        if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
            angle(angle<-100) = angle(angle<-100)+360;
            mangle = mean(angle,2);
            if mangle(1)>180
                mangle(1) = mangle(1)-360;
            end
        end
        xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, mangle, fmcw.f0, fmcw.c0));
    end
    % Radar RX Signal Processing
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

