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
        InitPos = [0,0,0]; %Position at start of measurement
        egoMotion = 0; %[true, false, XX(default)] car with radar moves
        
        %Antennas & Propagation
        TXpeakPower = 0.01; %10dBm in W
        TXgain = 17;
        RXgain = 15;
        RXNF = 10;
        RXant = 8;
        
        printNoiseCharacteristics = false; %Print the SNR and noise level
        %Noise and Clutter //ADJUST ACCORDING TO THE MEASUREMENT NOISE FLOOR
        NoiseFloor = -130; %dB (+/-dynamicNoise +RXNF +dBoffset)
        dynamicNoise = 10; %dB, +/- NoiseFloor
        backscatterStatClutter = false;
        numStatTargets = 60; % Rayleigh Mean for the number of static clutter targets
        dBoffset = 30; % offset for RD map plot
        
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
        chirp = []; %TX signal
        
        %Initialized in function:   generateAntPattern(obj)
        VantPattern = [];
        HantPattern = [];
        
        %Initialized in function:   setupMeasurement(obj)
        MSchan = [];
        MSradarplt = [];
        MStrx = [];
        MSRXarray = [];
        MSrcvx = [];
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
            if obj.egoMotion == false
                obj.egoMotion = 0; %stationary radar
            elseif obj.egoMotion == true
                obj.egoMotion = obj.velBins(end)* rand(); %random ego-motion of the carrying car
            end
        end
        
        
             
        %% Specify chirp sequence
        function obj = generateChirpSequence(obj)
            % output: add object feature obj.chirps which contains the
            % samples for one chirp sequence (chirpInterval) sampled with the
            % propagation sampling rate obj.Propagation_fs = obj.sweepBw
            % (avoiding undersampling occuring for lower fs).
            
            showplot = false; %for plot, change obj.Propagation_fs
            obj.Propagation_fs = obj.sweepBw; %sampling rate of the modelled prop signal
            
            if strcmp(obj.chirpShape, 'TRI')
                %TRIANGLE
                tup     = (0:obj.K-1) / obj.fs;
                tdown   = (obj.K:2*obj.K-1) / obj.fs;
                signal  = zeros(1,2*obj.K);
                signal(1:obj.K-1)    = exp(1i*2*pi *  obj.sweepBw/2/obj.chirpTime * tup.^2);
                signal(obj.K:end)    = exp(1i*2*pi * -obj.sweepBw/2/obj.chirpTime * tdown.^2)
            elseif strcmp(obj.chirpShape, 'SAW1')
                %SAWTOOTH    
                tup     = (0:obj.K-1) / obj.fs;
                signal  = zeros(1,obj.K);
                signal               = exp(1i*2*pi *  obj.sweepBw/2/obj.chirpTime * tup.^2);
                
            elseif strcmp(obj.chirpShape, 'SAWgap')
                %SAWTOOTH with gap
                tup     = (0:obj.K-1) / obj.fs;
                signal  = zeros(1,obj.chirpTime*obj.fs);
                signal(1:obj.K)    = exp(1i*2*pi *  obj.sweepBw/2/obj.chirpTime * tup.^2);
            else
                error('\n !!Unknown chirp sequence!!\nChoose from [TRI, SAW1, SAWgap].\n\n')
            end
            
            
            obj.chirp = signal;
                
            if showplot
                
                figure;
                plot(obj.chirps);
                figure;
                nfft = obj.ChirpInterval/obj.fs;
                spectrogram(obj.chirps, nfft, obj.fs) %
                title('Chirp sequence')
                
            end
            
        end
        
        
        %% Generate antenna pattern
        function obj = generateAntPattern(obj)
            az = -180:10:180;
            Vpattern = [-17, -13,-15, NaN,-17, -11,-8.5, -7.5, -5, -3.5,0,5,7,10,11,12,13.5,14,15,14,13.5,11,12,10,7,5,0,-3.5,-5,-7.5,-8.5,-11,-17,NaN,-15,-13,-17];
            Hpattern = [-17,NaN,NaN,NaN,NaN,NaN,NaN,-20, -17, -14.5,-12,-11,-15,-10,NaN, -10,-4,7,15,7,-4,-10,NaN,-10,-15,-11,-12,-14.5,-17,-20,NaN,NaN,NaN,NaN,NaN,NaN, -17];
            obj.VantPattern = repmat(Vpattern, [181,1]);
            obj.HantPattern = repmat(Hpattern, [181,1]);
        end
        
        
        %% Measurement Channel
        %   Full Prx = MSchan(txsignal,range)*sigma * G_tx * G_rx 
        function rxsignal = MSchan(obj, txsignal, range)
            % Radar equation with free space attenuation 
            lambda    = obj.c0/obj.f0;
            att       = lambda^2/((4*pi)^3*range^4);
            rxsignal  = att* txsignal;
        end
        
        %% Radar Platform
        function [pos, vel, axt] = move(obj, tsamp, acc = [0,0,0])
            % Calculates the current target position and velocity after 
            % moving the platform for a duration tsamp from the original position at tsamp=0.
            vel  = [obj.egoMotion, 0, 0];
            pos  = obj.InitPos + tsamp*vel + 0.5*acc*tsamp^2;
            axt  = [1,0,0];
        end
          
        
        
        %% Setup Measurement
        function obj = setupMeasurement(obj)
            % Radar Transmitter
            obj.MStrx = phased.Transmitter('PeakPower',obj.TXpeakPower,'Gain',obj.TXgain);
            % Radar Receiver
            antenna = phased.CustomAntennaElement('AzimuthAngles', -180:10:180, 'SpecifyPolarizationPattern', true, 'HorizontalMagnitudePattern', obj.HantPattern,'VerticalMagnitudePattern', obj.VantPattern);
            obj.MSRXarray = phased.ULA('Element', antenna, 'NumElements', obj.RXant, 'ElementSpacing', obj.c0/obj.f0, 'ArrayAxis', 'y');
            obj.MSrcvx = phased.ReceiverPreamp('Gain',obj.RXgain,'NoiseFigure',obj.RXNF);
        end
        
        
        %% Add Gaussian noise with varying standard deviation to the signal
        function s_beatnoisy = addGaussNoise(obj, s_beat)
            % Input:    s_beat (dimension of KxL)
            % obj.SNR is determining the amplitude of additional noise
            % obj.printNoiseCharacteristics = true; %DEBUG: print SNR and Noise Lv
            
            %Adjust Noise Floor to match FFT outputs in measurements
            FFToffset = 10; %dB
            
            %Apply offset and add random dynamics to Noise Floor
            dynOffset = ((rand()-0.5)*2)*obj.dynamicNoise;
            FFTnoiseFloor = obj.NoiseFloor + dynOffset + FFToffset; %
            
            %Calculate the noise power from the noise floor
            Pn = 10^(FFTnoiseFloor/10);
            
            %Compare to Beat Signal Power
            if strcmp(obj.chirpShape,'SAWgap')
                Ps = 1/(size(s_beat,1)/2)*sum(abs(s_beat(1:size(s_beat,1)/2,end,1).^2));
            else
                Ps = 1/(size(s_beat,1))*sum(abs(s_beat(:,end,1).^2));
            end
            if Pn>Ps*10 && Ps>0 && obj.printNoiseCharacteristics
                fprintf('Caution: Noise Power %.2f dB exceeds RX beat signal power %.2f dB!\nConsider setting a lower Noise Floor. \n', FFTnoiseFloor, 10*log10(Ps))
            end
            
            %Initialize Noise statistics
            sig = sqrt(Pn);
            mean = 0;
            sz = size(s_beat);
            noise = sig * randn(sz)+mean;
            
            noiseR = noise;
            %noiseR = sig * randn(sz)+mean;
            for a = 1:size(s_beat,3)
                FN = fftshift(fft(noiseR(:,:,a),[],1),1);
                FNR = zeros(size(FN));
                for c = 1:size(FNR,2)
                    % Difference of Signal power (R) is -10dB = 10^-1
                    FNR(:,c) = FN(:,c) .* [fliplr(10.^(-1*obj.rangeBins/obj.rangeBins(end))), 10.^(-1*obj.rangeBins/obj.rangeBins(end))]' ;
                end
                noiseR(:,:,a) = ifft(ifftshift(FNR,1),[],1);
            end
            RnoiseFaktor = 0.5+rand()*2;
            
            
            % Add Noise and range dep. Clutter to signal
            if obj.printNoiseCharacteristics
                SNR = Ps/Pn;
                fprintf('Generated gaussian Noise Floor at %.2f dB with Range dependency factor %.2f.\nSNR is %.5f (%.2f dB).\n', obj.NoiseFloor+dynOffset, RnoiseFaktor, SNR, 10*log10(SNR));
            end
            s_beatnoisy = s_beat+ (noise+noiseR*RnoiseFaktor);
        end
        
        
        
        %% Add static Clutter
        function sbC = addStaticClutter(obj,s_beat)
            % Faster Performance than backscatter targets
            % Pclutter_min = obj.NoiseFloor+10; %dB
            
%             %Adjust Noise Floor to match FFT outputs
%             FFToffset = -60; %dB
            
            % Init random Number of static Targets
            statTarg = ceil(raylrnd(1)*obj.numStatTargets); %generate a random amount of static targets
            %statTarg ~ [1, 60] targets on 160 Range bins
            
            
            % Random distances
            Ridx = floor(rand(statTarg,1)* length(obj.rangeBins))+1;
            Ridx(Ridx>length(obj.rangeBins)) = ceil(rand()*length(obj.rangeBins));
            ranges = obj.rangeBins(Ridx);
            
            % Rayleigh amplitudes with margin [NoiseFloor, NoiseFloor+20dB]
            AmpMargin = 15; %dB variance of Amplitude
            rAmp = obj.NoiseFloor - AmpMargin * raylrnd(ones(1,statTarg))+20;
            rangeAtt = 2*AmpMargin * (ranges/1.5/obj.rangeBins(end)); % Decay of clutter Power with Range ~R^2
            rAmp = sqrt(10.^((rAmp-rangeAtt)/10)); %Amp = sqrt(Pn), dB->scalar
            
            % Generate signals for clutter
%             fR = - obj.sweepBw/obj.chirpTime * 2/obj.c0 *ranges;
%             fd = -obj.f0 * 2/obj.c0 * 0;
%             phase = obj.f0*2/obj.c0*ranges;
            clutter = zeros(obj.K,obj.L, size(s_beat,3));
            
            rangeIdxMat = transpose(meshgrid(0:obj.K-1, 1:obj.L)); %Matrix 0:K Samples(time) in L (Chirps) columns
            dopplerIdxMat = meshgrid(0:obj.L-1, 1:obj.K); %Matrix 0:L in K lines

            for target = 1:statTarg
                azimut = rand()*180-90; % set random incident angle for static clutter
                for R = -obj.dR:obj.dR:+obj.dR
                    %for v = -obj.dV:obj.dV:obj.dV %Optional to add some N
                    for v = 0
                        fR = - obj.sweepBw/obj.chirpTime * 2/obj.c0 *(ranges(target)+R);
                        fd = obj.f0 * 2/obj.c0 * (v); % obj.egoMotion+v
                        phase = obj.f0*2/obj.c0*(ranges(target)+R);
                        % add signals for Frequency shifts (Range, Doppler and Phase shift)
                        rangeMat   = exp(1j*2*pi * fR/obj.fs * rangeIdxMat); % t = over K Sample values(time) add f_R, to each Chirp (L lines)
                        dopplerMat = exp(1j*2*pi * fd*obj.chirpInterval* dopplerIdxMat); % t = over L Chirps add f_D, to each Sample (lines)
                        phaseMat   = exp(1j*2*pi * phase); %phase difference due to Range

                        if R==0 && v==0
                            ampScale = 1; %main peak in center
                        else
                            ampScale = (rand()*0.75); %sidelobes with lower amp
                        end
                        
                        %Add phase shift from antenna array
                        %clutter = clutter + rAmp(target)*ampScale* (rangeMat.*dopplerMat.*phaseMat);
                        dx = (obj.c0/obj.f0)/2 * sind(azimut);
                        for ant= 1:size(s_beat,3)
                            phaseShiftArray = exp(1j * 2*pi*obj.f0* (ant-1)*dx/obj.c0);
                            clutter(:,:,ant) = clutter(:,:,ant) + rAmp(target)*ampScale* (rangeMat.*dopplerMat.*phaseMat)* phaseShiftArray;
                        end
                    end
                end
                
            end
            
            sbC = s_beat + real(clutter);
        end
        
        
        
        %% Generate Range Doppler Map from given beat signal s_beat
        function RDmap = RDmap(obj, s_beat)
            % Input:    s_beat (Matrix KxLxA)
            sz = size(s_beat);
            RDmap = zeros(sz(1)/2,sz(2), sz(3));
            
%             for ant = 1:obj.RXant
%                 %Select one RX antenna
%                 sb = s_beat(:,:,ant);
% 
%                 if size(sb,1) ~= obj.K
%                     error('\nBeat signal has wrong size: [%d,%d], expected K,L: [%d,%d]!!\nExpected length of chirp K doesnt match the beat signal rows...\n\n', size(sb,1), size(sb,2), obj.K, obj.L)
%                 elseif size(sb,2) ~= obj.L
%                     error('\nBeat signal has wrong size: [%d,%d], expected K,L: [%d,%d]!!\nExpected length of chirp sequence L doesnt match the beat signal columns...\n\n', size(sb,1), size(sb,2), obj.K, obj.L)
%                 end

            % 1st FFT for RANGE
            winK = repmat(hann(obj.K), [1,obj.L,obj.RXant]);
            scaWinK     = sum(winK(:, 1, 1));
            s_beat = cat(1,zeros(0, obj.L, obj.RXant), s_beat .* winK, zeros(0, obj.L, obj.RXant));
            SB = fft(ifftshift(s_beat,1) , obj.K, 1) /scaWinK; % *2/2048
%             SB = fft(s_beat,[], 1)/obj.K; %/length(s_beat(:,1)) FFT 1 of every column with K time samples
            SB = fftshift(SB,1);
            
            %Visualization Corrections
            %SB = SB(1:obj.K/2,:,:);
            SB = SB(obj.K/2+1:end,:,:); % dont show negative Range 


            % 2nd FFT for DOPPLER
            winL = repmat(hann(obj.L)', [obj.K/2, 1, obj.RXant]);
            scaWinL     = sum(winL(1,:,1));
            SB = cat(2, zeros(obj.K/2, 0, obj.RXant), SB .* winL, zeros(obj.K/2, 0, obj.RXant));
            SB = fft(ifftshift(SB,2), obj.L, 2)/scaWinL;
%             SB = fft(SB, [], 2) /obj.L; % /256  FFT 2 of every row with L chirps
            SB = fftshift(SB,2);

            
            % 3rd FFT for Angle
            winN        = zeros(1,1,obj.RXant);
            winN(:)     = hann(obj.RXant);
            winN        = repmat(winN, [obj.K/2, obj.L, 1]);
            
            % Output 160x256x16
%             SB = cat(3, zeros(obj.K/2, obj.L, obj.RXant/2), SB .* winN, zeros(obj.K/2, obj.L, obj.RXant/2));
%             SB = fft(ifftshift(SB,3), obj.RXant*2, 3)/sum(winN(1,1,:));
            % Output 160x256x8
            SB = cat(3, zeros(obj.K/2, obj.L, 0), SB .* winN, zeros(obj.K/2, obj.L, 0));
            SB = fft(ifftshift(SB,3), obj.RXant, 3)/sum(winN(1,1,:));
            
            SB = fftshift(SB,3);
            SB = flip(SB, 3); %Reorder RX antennas RXchannel(1:8) = [+90,-90]
            
            
            RDmap = 10 * log10(abs(SB).^2); % RD map in logarithmic scale
            
            % OPTIONAL: RDmap preprocessing
            %Set max(RD) to 0
%             RDmap = RDmap - max(RDmap);
            %Set minimum to -150dB
%             RDmap(RDmap < -200) = -200;
            
        end
        
        
        
        %% Plot the Range Doppler map
        function plotRDmap(obj, RDmap, target, plotAntennas)
            % Input:    RDmap (from RDmap())
            
            if plotAntennas == 0 
                % Only plot range Doppler map (drop Azimuth component)
                % Get rid of first range bins
                rangeDoppler = RDmap(6:end, :,:);
                % Max. across angle bins
                rangeDoppler = max(rangeDoppler,[], 3);
                
                % Convert RD Spectrum to grey scale image, max. value -20, min. value
                % -85. Output values are within 0 and 1. Max. and min. values are
                % adjusted according to the data, so that they hava a dynamic range of
                % around 65 dB
                figure;
                x = obj.velBins; % +obj.egoMotion Speed with egoMotion
                y = obj.rangeBins(1:length(obj.rangeBins/2));
                imagesc(x*3.6,y,rangeDoppler+obj.dBoffset) % ADDED 60dB OFFSET ONLY FOR COMPARISION WITH REAL DATA
                set(gca,'YDir','normal')
                xlabel('Velocity (km/h)')
                ylabel('Range (m)')
                colorbar
                colormap jet
                title(['Range-Doppler map of Backscattered Power in dB'])

                
            else
                for ant = plotAntennas
                    RDmap_plt = RDmap(:,:,ant);
                    figure;
                    x = obj.velBins; % +obj.egoMotion Speed with egoMotion
                    y = obj.rangeBins(1:length(obj.rangeBins/2));
        %             y = [fliplr(y),y];
                    if ~isempty(target) %show current target position
                        % Transform target R and V to indices in RD map
                        [~,targetRidx] = min(abs(target(1)-obj.rangeBins));
                        [~,targetVidx] = min(abs(target(2)-obj.velBins));

                        RDmap_plt = insertMarker(RDmap_plt, [targetVidx, targetRidx]); % add Marker
                        %bRD = insertShape(bRD,'circle',[150 280 35],'LineWidth',5);
                        fprintf('Simulation of Target starting at Range %.2f m and radial Velocity %.2f m/s.\n', ...
                        target(1), target(2))
                    end
                    imagesc(x*3.6,y,RDmap_plt(1:end,:,1)+obj.dBoffset) % ADDED 60dB OFFSET ONLY FOR COMPARISION WITH REAL DATA
                    set(gca,'YDir','normal')
                    xlabel('Velocity (km/h)')
                    ylabel('Range (m)')
                    colorbar
                    colormap jet
                    title(['Contour of Backscattered Power at RX antenna ', num2str(ant), ' in dB'])
                end
            end
        end
        
    end
end

