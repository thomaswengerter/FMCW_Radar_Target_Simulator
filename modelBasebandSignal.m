function [sb,obstruction] = modelBasebandSignal(target, targetID, map, rPlt, fmcw)
%% modelSignal() Summary
%   This function takes the configured radar 'target' fmcwect (from phased
%   toolbox) as input and generates the radar response for the radar
%   settings specified in FMCWradar Object 'radar'
%
%   INPUTS
%   target:     target fmcwect
%   targetID:   1 Pedestrian, 2 Bicycle, >=3 Vehicle
%   map:        Obstruction map for covered target points
%   fmcw:       fmcw radar fmcwect

%   OUTPUT
%   sb:         baseband RX signal


%% Start Radar Measurement
% Collect sampled reflections within the current chirpInterval for L consecutive chirps
% Sampling frequency for propagation is Propagation_fs = sweepBw to avoid
% undersampling

%obstruction factor if target points are covered
obstruction = 0;


if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    tsamp = fmcw.chirpInterval; % timestep to move target & radar
    xTX = fmcw.MStrx(fmcw.chirps()); % Radar transmitter signal
    
    
    %Target Backscatter
    if ~isempty(target)

        % Looping through chirps

        %% Move Radar & Targets
        %fmcw.MSradarplt = phased.Platform('InitialPosition',[0;0;fmcw.height], ...
        %    'OrientationAxesOutputPort',true, 'InitialVelocity', [fmcw.egoMotion;0;0], 'Acceleration', [0;0;0], ...
        %    'MotionModel', 'Acceleration', 'AccelerationSource', 'Input port');
        %[posr,velr,~] = fmcw.MSradarplt(tsamp, [0;0;0]); % current Position of Radar
        posr = rPlt(:,1);
        velr = rPlt(:,2);

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
            ridx = ceil(sqrt((post(1,:)-posr(1)).^2+((post(2,:)-posr(2)).^2))./fmcw.dR);
            azi = round(atand((post(2,:)-posr(2))./(post(1,:)-posr(1))));
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
        NumScatterers = shape(end); % getNumScatters(target)
        if NumScatterers>0
            % Reflection point positions and radial velocities
            [range,~] = rangeangle(posr,post,axt); % Calc angle between Radar and Target
            angle = zeros(2,length(range));
            angle(1,:) = round(atand((post(2,:)-posr(2))./(post(1,:)-posr(1)))); %azimuth
            angle(2,:) = asind((post(3,:)-posr(3))./range); %elevation
            velheading = (velt-velr);
            radaxis = (post-posr);
            %velazi = atand(velheading(2,:)./velheading(1,:));
            %velD = sqrt(velheading(1,:).^2+velheading(2,:).^2)*cosd(angle(1,:)-velazi);

            % Vektor Projection
            %       C=(dot(A,B)/norm(B)^2)*B
            radvel = zeros(1,NumScatterers);
            for sct = 1:NumScatterers
                %radaxisnorm = radaxis(:,sct)/norm(radaxis(:,sct));
                %radvel(:,sct) = dot(velheading(:,sct), radaxis(:,sct))/norm(radaxis(:,sct)) .* radaxisnorm;
                radvel(sct) = dot(velheading(:,sct), radaxis(:,sct))/norm(radaxis(:,sct));
            end
                
            % Reflection Point RCS (scalar)
            if targetID>=3 %Vehicle
                RCS = target.CarTarget.MeanRCS;
            elseif targetID == 2 %Bicycle
                RCS = target.BicyclistTarget.MeanRCS;
            else %Pedestrian
                % *** TODO: *** Correct RCS sampling
                RCSped = load('PedestrianRCS.mat');
                RCS = zeros(NumScatterers,1);
                for refPoint = 1:NumScatterers
                    elev = round(angle(2,refPoint));
                    azi = round(angle(1,refPoint));
                    if elev<-90 || elev>90
                        error('Invalid elevation for Pedestrian reflection point')
                    elseif azi<-180 || azi>180
                        error('Invalid azimuth for Pedestrian reflection point')
                    end
                    RCS(refPoint) = RCSped.RCSped(elev+91, azi+181, refPoint);
                end
            end

            % Radar Equation for received Power (RCS)
            Prx = fmcw.TXpeakPower* 10^(fmcw.RXgain/10)* 10^(fmcw.TXgain/10) * (fmcw.c0/fmcw.f0)^2 * RCS'./((4*pi*range).^2 .*(4*pi*range.^2));% Radar Equation
            rAmp = sqrt(Prx); %Amp = sqrt(Pn)

            % Generate baseband signals for point reflectors:
            %             fR = - fmcw.sweepBw/fmcw.chirpTime * 2/fmcw.c0 *range;
            %             fd = -fmcw.f0 * 2/fmcw.c0 * 0;
            %             phase = fmcw.f0*2/fmcw.c0*range;
            sb = zeros(fmcw.K,fmcw.L, fmcw.RXant);

            rangeIdxMat = transpose(meshgrid(0:fmcw.K-1, 1:fmcw.L)); %Matrix 0:K Samples(time) in L (Chirps) columns
            dopplerIdxMat = meshgrid(0:fmcw.L-1, 1:fmcw.K); %Matrix 0:L in K lines

            for target = 1:NumScatterers
                azimut = rand()*180-90; % set random incident angle for static clutter
                for dR = 0 %-fmcw.dR:fmcw.dR:+fmcw.dR
                    %for v = -fmcw.dV:fmcw.dV:fmcw.dV %Optional to add some N
                    for dv = 0
                        fR = - fmcw.sweepBw/fmcw.chirpTime * 2/fmcw.c0 *(range(target)+dR);
                        fd = -fmcw.f0 * 2/fmcw.c0 * (dv+radvel(target)); % fmcw.egoMotion+v
                        phase = fmcw.f0*2/fmcw.c0*(range(target)+dR);
                        % add signals for Frequency shifts (Range, Doppler and Phase shift)
                        rangeMat   = exp(1j*2*pi * fR/fmcw.fs * rangeIdxMat); % t = over K Sample values(time) add f_R, to each Chirp (L lines)
                        dopplerMat = exp(1j*2*pi * fd*fmcw.chirpInterval* dopplerIdxMat); % t = over L Chirps add f_D, to each Sample (lines)
                        phaseMat   = exp(1j*2*pi * phase); %phase difference due to Range

                        if dR==0 && dv==0
                            ampScale = 1; %main peak in center
                        else
                            ampScale = (rand()*0.75); %sidelobes with lower amp
                        end

                        %Add phase shift from antenna array
                        %clutter = clutter + rAmp(target)*ampScale* (rangeMat.*dopplerMat.*phaseMat);
                        dx = (fmcw.c0/fmcw.f0)/2 * sind(azimut);
                        for ant= 1:fmcw.RXant
                            % *** TODO: *** Individual antenna element patterns
                            % From Datasheet: Vpattern = [-17, -13,-15, NaN,-17, -11,-8.5, -7.5, -5, -3.5,0,5,7,10,11,12,13.5,14,15,14,13.5,11,12,10,7,5,0,-3.5,-5,-7.5,-8.5,-11,-17,NaN,-15,-13,-17];
                            %                 Hpattern = [-17,NaN,NaN,NaN,NaN,NaN,NaN,-20, -17, -14.5,-12,-11,-15,-10,NaN, -10,-4,7,15,7,-4,-10,NaN,-10,-15,-11,-12,-14.5,-17,-20,NaN,NaN,NaN,NaN,NaN,NaN, -17];
                            %                 (NaN: <-20dB)
                            
                            phaseShiftArray = exp(1j * 2*pi*fmcw.f0* (ant-1)*dx/fmcw.c0);
                            sb(:,:,ant) = sb(:,:,ant) + rAmp(target)*ampScale* (rangeMat.*dopplerMat.*phaseMat)* phaseShiftArray;
                        end
                    end
                end

            end

            sb =  real(sb);

%                 fmcw.MSchan.release();
%                 xtrans = fmcw.MSchan(repmat(xTX,1,N),posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
%                 RXsig = reflect(target,xtrans,angle); %receive sum of Reflections from target
% 
%                 mangle = mean(angle,2); %angle between target and radar
%                 if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
%                     angle(angle<-100) = angle(angle<-100)+360;
%                     mangle = mean(angle,2);
%                     if mangle(1)>180
%                         mangle(1) = mangle(1)-360;
%                     end
%                 end
% 
%                 % TODO: Check why mangle required for other targets????
%                 if targetID >= 2
%                     angle(angle>180) = angle(angle>180)-360;
%                     xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, angle, fmcw.f0, fmcw.c0)); 
%                 else
%                     xRX(:,chirp,:) = fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXsig, mangle, fmcw.f0, fmcw.c0)); 
%                 end
        end
    end

%     
%     %% NOT RECOMMENDED: Backscatter Static Clutter Scatterers
%     % Rather use function fmcw.addStaticClutter(s_beat) for better
%     % simulation performance
%     
%     % Add random static clutter Targets
%     maxClutterObjects = 30;
%     numClutterObjects = ceil(rand()*maxClutterObjects);
% 
%     cRX = zeros(size(xRX));
%     if fmcw.backscatterStatClutter
%         for fmcw = 1:numClutterObjects
%             Rfmcw = rand()*fmcw.rangeBins(end)+fmcw.dR; %static Clutter Range
%             Azfmcw = rand()*180-90;
%             Elfmcw = rand()*120-60;
%             targetpos = phased.Platform('InitialPosition',[Rfmcw*cos(Azfmcw)*cos(Elfmcw); Rfmcw*sin(Azfmcw)*cos(Elfmcw); Rfmcw*sin(Elfmcw)], ...
%                     'OrientationAxesOutputPort',true, 'InitialVelocity', [0;0;0], 'Acceleration', [0;0;0]);
%             [post,velt,axt] = targetpos(tsamp);
% 
%             Rayleigh = raylrnd(1:100);
%             Pdistribution = Rayleigh/max(Rayleigh);
%             sigma = 1- Pdistribution(ceil(rand()*100)); %RCS
%             ClutterObj = phased.RadarTarget('Model','Nonfluctuating','MeanRCS',sigma,...
%                     'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
% 
%             for chirp = 1:fmcw.L
%                 %[posr,velr,~] = fmcw.MSradarplt(tsamp);
%                 % RADAR POSITIONING NOT IMPLEMENTED CORRECTLY
%                 [~,angle] = rangeangle(posr,post,axt);
%                 mangle = mean(angle,2); %angle between target and radar
%                 if abs(mangle(1)-angle(1,1))>100 && abs(mangle(1)-angle(1,end))>100
%                     angle(angle<-100) = angle(angle<-100)+360;
%                     mangle = mean(angle,2);
%                     if mangle(1)>180
%                         mangle(1) = mangle(1)-360;
%                     end
%                 end
%                 fmcw.MSchan.release();
%                 xtrans = fmcw.MSchan(xTX,posr,post,velr,velt); %Signal transmission with incident sig for each scatterer
%                 RXclut = ClutterObj(xtrans,true); %receive sum of Reflections from target
%                 cRX(:,chirp,:) = reshape(cRX(:,chirp,:), size(cRX,1),size(cRX,3)) + fmcw.MSrcvx(collectPlaneWave(fmcw.MSRXarray, RXclut, mangle, fmcw.f0, fmcw.c0));
%             end                       
% 
%         end
%     end
%     
%     %% Radar RX Signal Processing
%     xRX = xRX + cRX; %add target and clutter signals
%     sb = zeros(size(xRX));
%     for ant = 1:fmcw.RXant
%         sb(:,:,ant) = conj(dechirp(xRX(:,:,ant), xTX)); %Mix Tx with Rx to transform sRx to baseband signal       
%     end
%     
%     % AD Downsampling fs
%     idxstep = fmcw.Propagation_fs/fmcw.fs;
%     sb = sb(1:idxstep:end,:,:);
    
    
%     %% Crop TX pause between chirps
%     if strcmp(fmcw.chirpShape,'SAWgap') && sb(fmcw.K+1,1,1)>0
%         % Check for pause after chirp
%         error('\nAre you sure you selected the correct Chirp Shape? No gap between SAW chirps detected\n\n')
%     else
%         % discard the TX pause from the signal
%         sb = sb(1:fmcw.K,:,:);
%     end
%Received complex signal sb
end
end

