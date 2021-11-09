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
sb = zeros(fmcw.K,fmcw.L, fmcw.RXant);

if strcmp(fmcw.chirpShape,'SAWgap')||strcmp(fmcw.chirpShape, 'TRI')||strcmp(fmcw.chirpShape,'SAW1')
    
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
        
        % Restore all Reflection Points and move model one timestep
        target = restoreReflectionPoints(target, fmcw.c0, fmcw.f0);
        [post,velt,axt] = move(target, 0); % read target Scatterer Positions, Velo and Ori
        
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
            if sum(sum(CoveredFilter))>0

                obstruction = 1; % some Target points are obstructed
                if sum(sum(CoveredFilter)) == length(post)
                    obstruction = 4; % all points hidden
                elseif sum(sum(CoveredFilter))>(3*length(post)/4)
                    obstruction = 3; % more than 3/4 of the target points are obstructed
                elseif sum(sum(CoveredFilter))>(length(post)/2)
                    obstruction = 2; % more than 1/2 of the target points are obstructed
                end
                target = RemoveHiddenScatterers(target, CoveredFilter, fmcw.c0, fmcw.f0);
            end
        end
        

        %% Reflect Signal from Scatterers
        shape = size(post);
        NumScatterers = shape(end); % getNumScatters(target)
        if NumScatterers>0
            % Reflection point positions and radial velocities
            range = sqrt((posr(1)-post(1,:)).^2+(posr(2)-post(2,:)).^2+(posr(3)-post(3,:)).^2);
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
            RCS = target.MeanRCS;
                
            % Radar Equation for received Power (RCS)
            Prx = fmcw.TXpeakPower* 10^(fmcw.RXgain/10)* 10^(fmcw.TXgain/10) * (fmcw.c0/fmcw.f0)^2 * RCS./((4*pi*range).^2 .*(4*pi*range.^2));% Radar Equation
            rAmp = sqrt(Prx); %Amp = sqrt(Pn)

            % Generate baseband signals for point reflectors:
            %             fR = - fmcw.sweepBw/fmcw.chirpTime * 2/fmcw.c0 *range;
            %             fd = -fmcw.f0 * 2/fmcw.c0 * 0;
            %             phase = fmcw.f0*2/fmcw.c0*range;

            rangeIdxMat = transpose(meshgrid(0:fmcw.K-1, 1:fmcw.L)); %Matrix 0:K Samples(time) in L (Chirps) columns
            dopplerIdxMat = meshgrid(0:fmcw.L-1, 1:fmcw.K); %Matrix 0:L in K lines

            for target = 1:NumScatterers
                if RCS(target) > 0
                    azimut = angle(1,target); % incident angle for reflection point
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
            end

            sb =  real(sb);

        end
    end

end
end

