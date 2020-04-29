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
        
        TXpeakPower = 0.01; %10dBm in W
        TXgain = 17;
        RXgain = 15;
        RXNF = 0;
        NoiseFloor = -70; %dB
        dynamicNoise = 10; %dB, +/- NoiseFloor
        
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
            obj.rangeBins = (0:obj.K/2-1)*obj.dR;
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
        
        
        
        %% Add Gaussian noise with varying standard deviation to the signal
        function s_beatnoisy = addGaussNoise(obj, s_beat)
            % Input:    s_beat (dimension of KxL)
            % fmcw.SNR is determining the amplitude of additional noise
            printNoiseCharacteristics = true; %DEBUG
            
            %Adjust Noise Floor to match FFT outputs
            FFToffset = -60; %dB
            
            %Apply offset and add random dynamics to Noise Floor
            dynOffset = ((rand()-0.5)*2)*obj.dynamicNoise;
            FFTnoiseFloor = obj.NoiseFloor+FFToffset + dynOffset;
            
            %Calculate the noise power from the noise floor
            Pn = 10^(FFTnoiseFloor/10);
            
            %Compare to Beat Signal Power
            if strcmp(obj.chirpShape,'SAWgap')
                Ps = 1/(size(s_beat,1)/2)*sum(abs(s_beat(1:size(s_beat,1)/2,end).^2));
            else
                Ps = 1/(size(s_beat,1))*sum(abs(s_beat(:,end).^2));
            end
            if Pn>Ps*10 && Ps>0 && printNoiseCharacteristics
                fprintf('Caution: Noise Power %.2f dB exceeds RX beat signal power %.2f dB!\nConsider setting a lower Noise Floor. \n', FFTnoiseFloor, 10*log10(Ps))
            end
            
            %Initialize Noise statistics
            sig = sqrt(Pn);
            mean = 0;
            sz = size(s_beat);
            noise = sig * randn(sz)+mean;
            
            noiseR = sig * randn(sz)+mean;
            FN = fftshift(fft(noiseR,[],1),1);
            FNR = zeros(size(FN));
            for c = 1:size(FNR,2)
                % Difference of Signal power (R) is -10dB = 10^-1
                FNR(:,c) = FN(:,c) .* [fliplr(10.^(-1*obj.rangeBins/obj.rangeBins(end))), 10.^(-1*obj.rangeBins/obj.rangeBins(end))]' ;
            end
            noiseR = ifft(ifftshift(FNR,1),[],1);
            RnoiseFaktor = rand()*3;
            
            
            % Add Noise and range dep. Clutter to signal
            if printNoiseCharacteristics
                SNR = Ps/Pn;
                fprintf('Generated gaussian Noise Floor at %.2f dB with Range dependency factor %.2f.\nSNR is %.5f (%.2f dB).\n', obj.NoiseFloor+dynOffset, RnoiseFaktor, SNR, 10*log10(SNR));
            end
            s_beatnoisy = s_beat+ (noise+noiseR*RnoiseFaktor);
        end
        
        
        
        %% Add Weibull noise
        function s_beatnoisy = addWeibullNoise(obj,s_beat)
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
            
            SB = fft(s_beat,[], 1); %/length(s_beat(:,1)) FFT 1 of every column with K time samples
            SB = fftshift(SB,1);
            
%             % DEBUG plot
%             % Show first FFT with Range resolution
%             figure
%             x = -length(s_beat(:,1))/2*obj.dR:obj.dR:length(s_beat(:,1))/2*obj.dR-obj.dR;
%             plot(x, SB(:,10)) %FFT of 10th chirp
            
            
            SB = fft(SB, [], 2); % /256  FFT 2 of every row with L chirps
            SB = fftshift(SB,2);
            
            %Visualization Corrections
            SB = SB(length(s_beat(:,1))/2+1:end,:); % dont show negative Range
            %SB = fliplr(SB); % switch sign of Doppler Velocity 
            
            RDmap = 10 * log10(abs(SB).^2); % RD map in logarithmic scale
            
            %% RDmap preprocessing
            %Set max(RD) to 0
%             RDmap = RDmap - max(RDmap);
            %Set minimum to -150dB
%             RDmap(RDmap < -200) = -200;
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
                [~,targetRidx] = min(abs(target(1)-obj.rangeBins));
                [~,targetVidx] = min(abs(target(2)-obj.velBins));

                RDmap_plt = insertMarker(RDmap_plt, [targetVidx, targetRidx]); % add Marker
                %bRD = insertShape(bRD,'circle',[150 280 35],'LineWidth',5);
                fprintf('Simulation of Target starting at Range %d m and radial Velocity %d m/s.\n', ...
                target(1), target(2))
            end
            imagesc(x,y,RDmap_plt(:,:,1))
            set(gca,'YDir','normal')
            xlabel('Velocity in m/s')
            ylabel('Range in m')
            colorbar
            title('Contour of Backscattered Power in dB')
            
        end
        
    end
end

