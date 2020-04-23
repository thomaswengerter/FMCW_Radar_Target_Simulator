classdef FMCWradar
    %% FMCWradar class
    %   Contains relevant properties of radar device
    %   Functions: 
    %   Call to initialize class
    %       obj = init_RDmap(obj): initialize Range Doppler (RD) map axis
    %       obj = generateChirpSequence(obj): initialize FMCW chirp samples
    %   s_beatnoisy = addGaussNoise(s_beat): add Gaussian noise to signal s_beat
    %   RDmap = RDmap(obj, s_beat): calc RD map (in dB) from s_beat by 2xFFT
    
    
    %ENTER FMCW SPECS HERE
    properties
        chirpShape = 'SAWgap'; % Select chirp shape: ['TRI', 'SAW1', 'SAWgap']
        sweepBw = 1e9; %slope of chirp ramp
        chirpTime = 32e-6; %Duration of a chirp ramp fmin->fmax
        
        c0 = []; %Propagation speed of radar rays in Air
        fs = 10e6; %ADC sample frequency
        f0 = 76.5e9; %Radar frequency
        chirpsCycle = 256; %Number of chirps per measurement cycle (Doppler)
        height = 0.5; %position above street in m
        
        TXpeakPower = 0.1; %10dBm in W
        TXgain = 17;
        RXgain = 15;
        RXNF = 0;
        
        %Initialized in function:   init_RDmap(obj)
        chirpInterval = [];
        K = []; % #Range samples
        L = []; % #Velocity samples
        dR = []; % Range resolution
        dV = []; % Velocity resolution
        rangeBins = []; % Range Bins RD map
        velBins = []; % Velocity Bins RD map
        Rmaxsamp = []; % max unambiguous Range (limited by fs)
        
        %Initialized in function:   generateChirpSequence(obj)
        Propagation_fs = []; %sampling rate of the modelled signal
        chirps = []; %object of class phased.FMCWwaveform or phased.LinearFMWaveform
    end
    
    
    
    methods
        %% Initialize Resolution of Range Doppler Map
        function obj = init_RDmap(obj)
            %   K: Number of Range Bins
            %   L: Number of Velocity Bins
            %   dR: Resolution in Range
            %   dV: Resolution in Velocity
            %   rangeBins: y-axis of RD map
            %   velBins: x-axis of RD map
            global c_0;
            obj.c0 = c_0;
            
            if strcmp(obj.chirpShape, 'SAWgap')||strcmp(obj.chirpShape, 'TRI')
                obj.chirpInterval = 2*obj.chirpTime; %Interval of one completed chirp+recovery time
            else
                obj.chirpInterval = obj.chirpTime; %Interval of one completed chirp
            end
            
            obj.K = obj.chirpTime * obj.fs;  %320 samples per receiving chirpTime (discard negative Ranges)
            obj.L = obj.chirpsCycle;  %Number of chirps observed in one cycle
            obj.dR = obj.c0/(2*obj.sweepBw); %Range resolution
            obj.dV = 1/(obj.L *obj.chirpInterval) * obj.c0 /(2*obj.f0); %Velocity resolution
            obj.rangeBins = (1:obj.K/2-1)*obj.dR;
            obj.velBins = (-obj.L/2:obj.L/2-1) *obj.dV;
            %RmaxChirp = (chirpInterval-chirpTime)*c0/2; %max unabiguous Range limited by receiving interval
            obj.Rmaxsamp = obj.fs/4*obj.c0*obj.chirpTime/obj.sweepBw; %max Range limited by sampling freq

        end
        
        
        
        %% Specify chirp sequence
        function obj = generateChirpSequence(obj)
            % output: add object feature obj.chirps which contains the
            % samples for one chirp sequence (chirpInterval) sampled with the
            % propagation sampling rate obj.Propagation_fs = obj.sweepBw
            % (avoiding undersampling occuring for lower fs).
            
            showplot = false; %for plot, change obj.Propagation_fs
            obj.Propagation_fs = obj.sweepBw; %sampling rate of the modelled prop signal
            
            
            %TRIANGLE
            if strcmp(obj.chirpShape,'TRI')
                obj.chirps = phased.FMCWWaveform('SampleRate',obj.Propagation_fs,'SweepTime',obj.chirpTime, ...
                    'SweepBandwidth',obj.sweepBw,...
                    'SweepDirection', 'Triangle',... 
                    'SweepInterval', 'Positive', 'NumSweeps', 1);
            
            %SAWTOOTH
            elseif strcmp(obj.chirpShape,'SAW') %256 chirps
                obj.chirps = phased.FMCWWaveform('SampleRate',obj.Propagation_fs,'SweepTime',obj.chirpTime, ...
                    'SweepBandwidth',obj.sweepBw,...
                    'SweepDirection', 'Up',...  %SweepDirection Triangle
                    'SweepInterval', 'Positive', 'NumSweeps', obj.L);            
            
            elseif strcmp(obj.chirpShape,'SAW1') %1 chirp
                obj.chirps = phased.FMCWWaveform('SampleRate',obj.Propagation_fs,'SweepTime',obj.chirpTime, ...
                    'SweepBandwidth',obj.sweepBw,...
                    'SweepDirection', 'Up',...  %SweepDirection Triangle
                    'SweepInterval', 'Positive', 'NumSweeps', 1);            
            
                
            %SAWTOOTH with gap
            elseif strcmp(obj.chirpShape,'SAWgap')
                    obj.chirps = phased.LinearFMWaveform('SampleRate',obj.Propagation_fs,...
                        'SweepBandwidth',obj.sweepBw, 'PulseWidth',obj.chirpTime,...
                        'DurationSpecification','Pulse width', 'PRF', 1/obj.chirpInterval, ...
                        'SweepInterval','Positive');
                    
            else
                fprintf('\n !!Did not recognize chirp sequence!!\n\n')
            end
                
            if showplot
                
                figure;
                plot(obj.chirps);
                figure;
                fsig = step(obj.chirps); %Samples of FMCW waveform
                windowlength = 32;
                noverlap = 16;
                nfft = 32;
                spectrogram(fsig,windowlength,noverlap,nfft,obj.chirps.SampleRate,'yaxis') %
                title('Chirp sequence')
                
                error('Show chirp plot and stop...')
                % Downsample chirp
                
            end
        end
        
        
        
        %% Calculate Baseband signal s_beat
        function s_beat = modelSignal(obj, amp, f_Range, f_Doppler, phase)
            RangeIdxMat = meshgrid(1:obj.K, 1:obj.L); %Matrix 0:K Samples(time) in L (Chirps) lines
            DopplerIdxMat = meshgrid(1:obj.L,1:obj.K); %Matrix 0:L in K lines
            
            rangeMat = exp(1j*2*pi * f_Range/obj.fs * RangeIdxMat); % t = over K Sample values(time) add f_R, to each Chirp (L lines)
            dopplerMat = exp(1j*2*pi * f_Doppler*obj.chirpIntervall* DopplerIdxMat); % t = over L Chirps add f_D, to each Sample (lines)
            phaseMat   = exp(1j*2*pi * phase); % phase difference due to Range
            
            
            s_beat = amp * (dopplerMat*rangeMat*phaseMat);
        end
        
        
        
        %% Add Gaussian noise with varying standard deviation to the signal
        function s_beatnoisy = addGaussNoise(s_beat)
            % Input:    s_beat (dimension of KxL)
            noiseStdList = 0.5:0.01:2;
            mean = 0;
            stdidx = floor(rand()*length(noiseStdList))+1;
            noiseStd = noiseStdList(stdidx);
            sz = size(s_beat);
            noise = noiseStd * randn(sz)+mean;
            
            s_beatnoisy = s_beat+noise;
        end
        
        
        
        %% Add Weibull noise
        function s_beatnoisy = addWeibullNoise(s_beat)
            s_beatnoisy = s_beat;
        end
        
        
        
        %% Generate Range Doppler Map from given beat signal s_beat
        function RDmap = RDmap(obj, s_beat)
            % Input:    s_beat (Matrix KxL)
            
            if size(s_beat,1) ~= obj.K
                error('\nBeat signal has wrong size: [%d,%d], expected K,L: [%d,%d]!!\nExpected length of chirp K doesnt match the beat signal rows...\n\n', size(s_beat,1), size(s_beat,2), obj.K, obj.L)
            elseif size(s_beat,2) ~= obj.L
                error('\nBeat signal has wrong size: [%d,%d], expected K,L: [%d,%d]!!\nExpected length of chirp sequence L doesnt match the beat signal columns...\n\n', size(s_beat,1), size(s_beat,2), obj.K, obj.L)
            end
            
            SB = fft(s_beat,[], 1)/length(s_beat(:,1)); % FFT 1 of every column with K time samples
            SB = fftshift(SB,1);
            
            % DEBUG plot
            figure
            x = -length(s_beat(:,1))/2*obj.dR:obj.dR:length(s_beat(:,1))/2*obj.dR-obj.dR;
            plot(x, SB(:,10)) %FFT of 10th chirp
            
            
            SB = fft(SB, [], 2)/256; % FFT 2 of every row with L chirps
            SB = fftshift(SB,2);
            
            %Visualization Corrections
            SB = SB(length(s_beat(:,1))/2+1:end,:); % dont show negative Range
            %SB = fliplr(SB); % switch sign of Doppler Velocity 
            
            RDmap = 10 * log10(abs(SB).^2); % RD map in logarithmic scale
        end
        
        
        
        %% Plot the Range Doppler map
        function plotRDmap(obj, RDmap, target)
            % Input:    RDmap (from RDmap())
            RDmap_plt = RDmap;
            figure;
            x = obj.velBins;
            y = obj.rangeBins(1:length(obj.rangeBins/2));
%             y = [fliplr(y),y];
            if ~isempty(target) %show current target position
                % Transform target R and V to indices in RD map
                targetRidx = floor(target(1)/obj.dR); %Range idx are fliped
                targetVidx = (obj.L/2) + floor(target(2)/obj.dV);
                if targetRidx <= 0
                    % Range < 0m
                    targetRidx = 1; 
                elseif targetRidx>length(obj.rangeBins)
                    % Range > 24m too large
                    targetRidx = length(obj.rangeBins);
                end
                if targetVidx <= 0
                    targetVidx = 1;
                elseif targetVidx > obj.L
                    targetVidx = obj.L;
                end
                RDmap_plt = insertMarker(RDmap_plt, [targetVidx, targetRidx]); % add Marker
                %bRD = insertShape(bRD,'circle',[150 280 35],'LineWidth',5);
            end
            imagesc(x,y,RDmap_plt(:,:,1))
            set(gca,'YDir','normal')
            xlabel('Velocity in m/s')
            ylabel('Range in m')
            colorbar
            title('Contour of Backscattered Power in dB')
            fprintf('Simulation of Target starting at Range %d m and radial Velocity %d m/s.\n', ...
                target(1), target(2))
        end
        
    end
end

