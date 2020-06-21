%% POWER TEST
% Point target power return
clear
global c_0;
c_0 = 299792458;
plotAntennas = [1]; %list indices of RX antenna elements to be plotted in RD map

%Generate Radar Object
fmcw = FMCWradar;
fmcw = fmcw.init_RDmap();
% Initialize FMCW radar object and environment
fmcw = fmcw.generateChirpSequence(); %Generate chirp waveform, initialize fmcw.chirps
fmcw = fmcw.generateAntPattern(); %Generate antenna Pattern, initialize fmcw.antPattern
fmcw = fmcw.setupMeasurement(); %setup all measurement environment objects for 'modelSignal.m'

pts = 1;
xRX = complex(zeros(round(fmcw.chirpInterval*fmcw.Propagation_fs),fmcw.L, fmcw.RXant));
RCSsigma = 10^0.5; % -10dBsm
target = phased.RadarTarget('MeanRCS', ones(1,pts)*RCSsigma/pts,...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
Platform = phased.Platform('InitialPosition',[10*ones(1,pts); 0*ones(1,pts); 0.5*ones(1,pts)], ...
                    'OrientationAxesOutputPort',true, 'Velocity', [2*ones(1,pts); 0*ones(1,pts); 0*ones(1,pts)]);          

if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    tsamp = fmcw.chirpInterval; % timestep to move target & radar
    xTX = fmcw.MStrx(fmcw.chirps()); % Radar transmitter signal
    
    
    %Target Backscatter
    if ~isempty(target)
        for chirp = 1:fmcw.L
            % Looping through chirps
            [posr,velr,axr] = fmcw.MSradarplt(tsamp); % current Position of Radar
            [post,velt,axt] = Platform(tsamp); % move ped / bike
            
            [~,angle] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
            shape = size(post);
            N = shape(end); % getNumScatters(target)
            fmcw.MSchan.release();
            xtrans = fmcw.MSchan(repmat(xTX,1,N),posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
            RXsig = target(xtrans);

            
            % TODO: Check why mangle required for other targets????
            angle(angle>180) = angle(angle>180)-360;
            xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, angle, fmcw.f0, fmcw.c0)); 
        end
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

% sbn = fmcw.addGaussNoise(sb);
pRD = fmcw.RDmap(sb);
fmcw.plotRDmap(pRD, [], plotAntennas);
