function [sb] = modelSignal(target, fmcw)
%% modelSignal() Summary
%   This function takes the configured radar 'target' object (from phased
%   toolbox) as input and generates the radar response for the radar
%   settings specified in FMCWradar Object 'radar'


% Generate chirp waveform
fmcw = fmcw.generateChirpSequence(); %initialize fmcw.chirps

% Channel Model
chan = phased.FreeSpace('PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0, ...
    'TwoWayPropagation',true,'SampleRate',fmcw.Propagation_fs);
% Radar Position and Motion
radarplt = phased.Platform('InitialPosition',[0;0;fmcw.height], ...
    'OrientationAxesOutputPort',true, 'InitialVelocity', [0;0;0], 'Acceleration', [0;0;0]);
% Radar Transmitter
trx = phased.Transmitter('PeakPower',fmcw.TXpeakPower,'Gain',fmcw.TXgain);
% Radar Receiver
RXarray = phased.ULA('NumElements', fmcw.RXant, 'ElementSpacing', fmcw.c0/fmcw.f0);
rcvx = phased.ReceiverPreamp('Gain',fmcw.RXgain,'NoiseFigure',fmcw.RXNF);



%% Start Radar Measurement
% Collect sampled reflections within the current chirpInterval for L consecutive chirps
% Sampling frequency for propagation is Propagation_fs = sweepBw to avoid
% undersampling
xRX = complex(zeros(round(fmcw.chirpInterval*fmcw.Propagation_fs),fmcw.L, fmcw.RXant));

if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    tsamp = fmcw.chirpInterval; % timestep to move target & radar
    xTX = trx(fmcw.chirps()); % Radar transmitter signal
    for chirp = 1:fmcw.L
        % Looping through chirps
        [posr,velr,axr] = radarplt(tsamp); % current Position of Radar
        %tseq = fmcw.chirpsCycle*fmcw.chirpInterval; % Duration of radar measurement
        [post,velt,axt] = move(target,tsamp,target.InitialHeading); % start Moving target
        [~,angle] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
        shape = size(post);
        N = shape(end); % getNumScatters(target)
        xtrans = chan(repmat(xTX,1,N),posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
        RXsig = reflect(target,xtrans,angle); %receive sum of Reflections
        mangle = mean(angle,2); %angle between target and radar
        if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
            angle(angle<-100) = angle(angle<-100)+360;
            mangle = mean(angle,2);
            if mangle(1)>180
                mangle(1) = mangle(1)-360;
            end
        end
        xRX(:,chirp,:) = rcvx(collectPlaneWave(RXarray, RXsig, mangle, fmcw.f0, fmcw.c0));
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

    
elseif strcmp(fmcw.chirpShape,'SAW')
    %Full Chirp Sequence of 256 in one measurement
    %Macht eigentlich keinen Sinn... Besser SAW1!!
    tsamp = fmcw.chirpTime*fmcw.L; %timestep to move target & radar
    [posr,velr,axr] = radarplt(tsamp); % current Position of Radar
    %tseq = fmcw.chirpsCycle*fmcw.chirpInterval; % Duration of radar measurement
    [post,velt,axt] = move(target,tsamp,target.InitialHeading); % start Moving target
    [~,angle] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
    xTX = trx(fmcw.chirps()); % Radar transmitter signal
    shape = size(post);
    N = shape(end); % getNumScatters(target)
    xtrans = chan(repmat(xTX,1,N),posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
    RXsig = reflect(target,xtrans,angle);
    mangle = mean(angle,2); %angle between target and radar
    if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
        angle(angle<-100) = angle(angle<-100)+360;
        mangle = mean(angle,2);
        if mangle(1)>180
            mangle(1) = mangle(1)-360;
        end
    end
    xRX = rcvx(collectPlaneWave(RXarray, RXsig, mangle, fmcw.f0, fmcw.c0));
    
    % Radar RX Signal Processing
    sb = conj(dechirp(xRX, xTX)); %Mix Tx with Rx to transform sRx to baseband signal
    sb = imag(sb);
    sb = reshape(sb,[],fmcw.L);
    
    %AD Downsampling fs
    idxstep = fmcw.fs/fmcw.Propagation_fs;
    sb = sb(1:idxstep:end);
end

%Received complex signal sb
end

