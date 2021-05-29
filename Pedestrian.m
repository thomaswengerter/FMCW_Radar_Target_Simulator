classdef Pedestrian
    %CLASS BackscatterPedestrian class
    %   Generate prominent radar scattering points for a pedestrian in certain
    %   position. Consider different heights, moving/swinging body parts (micro 
    %   Doppler), angular positions of target. 
    
    properties
        plotContour = false; %bool: set to true to see scattering points
        
        ID = [];
        typeNr = [];
        xPos = []; %center y position
        yPos = []; %center x position
        heading = []; %angle of movement relative to x-axis
        WalkingSpeed = []; %velocity along heading angle
        width = []; % width
        length = []; % length
        Height = []; % height
        StepLength = []; % distance covered by one step
        StepDuration = []; % duration of one step
        RCS = []; %RCS real measurement data
        
        ReceptionAngle = 180; % SET MAX INCIDENT ANGLE RANGE FOR RAY RECEPTION [-ReceptionAngle/2, ReceptionAngle/2]
        ReflectionsPerContourPoint = 1; % SET THIS PARAMETER FOR RESOLUTION
        drefPoints = []; %Number of reflection points
        
        %for generateObstructionMap()
        aziCoverage = [];  %Azimut coverage
        rCoverage = []; %Range coverage
        leakingRatio = []; %Transparency
        
        RCSsigma = -6; %average total RCS of Pedestrians
        InitialHeading = []; %only Spaceholder for move() function call!
        Acceleration = []; %Acceleration for all move() steps
        N = []; %final Number of Scattering points
        TargetPlatform = []; %target platform
        PedestrianTarget = []; %target object
    end
    
    methods
        %% Initialize Dimensions
        function obj = initPedestrian(obj)
            obj.width = obj.Height/(2.5+rand()*1.3);
            obj.length = 0.25 + rand()* 0.25;
            obj.StepLength = obj.Height*0.3871;
            obj.StepDuration = obj.StepLength/obj.WalkingSpeed;
        end
        
        %% Convert Cathesian to Spherical Coordinates
        function [r,phi] = Sphere(~,x,y)
            r = sqrt(x^2+y^2); %range
            phi = atand(y/x); %angle in degree 
        end
        
        
        %% Normalize Angle to [-180,180]
        function nAngle = normAngle(~,angles)
            nAngle = zeros(size(angles));
            for i = 1:length(angles)
                angle = angles(i);
                while abs(angle)>180
                    if angle>180 
                        angle = angle - 360;
                    elseif angle < -180
                        angle = angle + 360;
                    end
                end
                nAngle(i) = angle;
            end
        end
        
        
        %% Coordinate System Transformation
        function [xPos, yPos] = toLocal(~, xi, yi, angle)
            relangle = angle+ atand(yi./xi);
            r = sqrt(xi.^2+yi.^2);
            xPos = cosd(relangle).*r;
            yPos = sind(relangle).*r;
        end
        
        
        %% Generate Point Modell with RCS
        function obj = generateBackscatterTarget(obj,fmcw,rPos)
            %   For current measurement setup, generate the Point Modell
            %   with corresponding RCS. Consider surface
            %   propagation, micro-Doppler and characteristic target 
            %   reflection points.
            
            
%             %Some Parameters
%             radarPos = [0,0,fmcw.height]; %static Radar position
%             azi = atand(obj.yPos/obj.xPos); %azimuth of target
%             R = sqrt(obj.xPos^2+obj.yPos^2); %range of target
            
            
            %--------------------------------------------------------------
            % Static Reflection Points 
            % Define static Reflection Point Locations in local coordinate system
            
            Contour = zeros(12,3);
            %Contour = [xRefPoint, yRefPoint, zRefPoint]
            
            
            %FR/FL
            xstart  =  2.5*obj.WalkingSpeed*obj.StepDuration/(2*pi);
            Contour(1,:) =  [-obj.StepLength/2, -obj.width/2.5, 0.1];
            Contour(2,:) =  [obj.StepLength/2, obj.width/2.5, 0.1];
            
            %KR/KL
            xstart  =  obj.WalkingSpeed*(1/2+1/(2*pi))*obj.StepDuration;
            Contour(3,:) =  [-obj.StepLength/3, -obj.width/2.5, obj.Height*0.294];
            Contour(4,:) =  [obj.StepLength/3, obj.width/2.5, obj.Height*0.294];
            
            %HR/HL
            xstart  =  obj.WalkingSpeed*(0.66/2+1/(2*pi))*obj.StepDuration;
            Contour(5,:) =  [-obj.StepLength/4.5, -obj.width/2, obj.Height*0.53];
            Contour(6,:) =  [obj.StepLength/4.5, obj.width/2, obj.Height*0.53];
            
            %ER/EL
            xstart  =  obj.WalkingSpeed*(0.33/2+1/(2*pi))*obj.StepDuration;
            Contour(7,:) =  [-obj.StepLength/4.5, -obj.width/2, obj.Height*0.65];
            Contour(8,:) =  [obj.StepLength/4.5, obj.width/2, obj.Height*0.65];
            
            %SR/SL
            Contour(9,:)  = [0, -obj.width/2, obj.Height*0.79];
            Contour(10,:) = [0, obj.width/2, obj.Height*0.79];
            
            %C/H
            Contour(11,:) = [0, 0, obj.Height*0.7];
            Contour(12,:) = [0, 0, obj.Height*0.95];
            
            Contour(:,1) = Contour(:,1) + obj.xPos;
            Contour(:,2) = Contour(:,2) + obj.yPos;
            
            % Transfer to global Coordinates
            [~,angl] = Sphere(obj, obj.xPos, obj.yPos);
            [Contour(:,1), Contour(:,2)] = toLocal(obj, Contour(:,1), Contour(:,2), angl);
            
            %Calculate RCS for Contour Points
            azi = atand(Contour(:,2)./Contour(:,1)); %azimuth of target Point
            DOA = obj.normAngle(azi-obj.heading-180); %hitting angle at Contour Point            
            
            
            % ASSIGN RCS VALUE
            relRCS = zeros(12,1);
            relRCS(1:2) = 10^-1.03;  %FR/FL
            relRCS(3:4) = 10^-0.38;  %KR/KL
            relRCS(5:6) = 10^-1.22;  %HR/HL
            relRCS(7:8) = 10^-0.82;  %ER/EL
            relRCS(9:10)= 0.1;       %SR/SL
            relRCS(11)  = 0.5;       %C
            relRCS(12)  = 0.3;       %H
            
            if obj.plotContour
                figure
                scatter(Contour(:,1),Contour(:,2),[], 'b')
                hold on;
            end
            
            
            %--------------------------------------------------------------
            % Micro-Doppler Velocities
            % Describe Walking Movements
            walkVelo = zeros(12,ceil(obj.StepDuration/fmcw.chirpInterval));
            t = 0:fmcw.chirpInterval:obj.StepDuration;
            %FR
            walkVelo(1,:) = 2.5*obj.WalkingSpeed* sin(2*pi*t/(obj.StepDuration));
            walkVelo(1,walkVelo(1,:)<0) = 0;
            %FL
            walkVelo(2,:) = 2.5*obj.WalkingSpeed* sin(pi+2*pi*t/(obj.StepDuration)); %-0.1*2*pi
            walkVelo(2,walkVelo(2,:)<0) = 0;
            %KR
            walkVelo(3,:) = obj.WalkingSpeed* (1+ sin(2*pi*t/(obj.StepDuration))+sin(2*pi*t/(obj.StepDuration*0.5))); % +sin(2*pi*t/(obj.StepDuration*0.5))
            %KL
            walkVelo(4,:) = obj.WalkingSpeed* (1+ sin(pi+2*pi*t/(obj.StepDuration))+sin(pi+2*pi*t/(obj.StepDuration*0.5)));
            %HR
            walkVelo(5,:) = obj.WalkingSpeed* (0.66+  sin(2*pi*t/(obj.StepDuration)));
            %HL
            walkVelo(6,:) = obj.WalkingSpeed* (0.66+  sin(pi+2*pi*t/(obj.StepDuration)));
            %ER
            walkVelo(7,:) = obj.WalkingSpeed* (0.33+  sin(2*pi*t/(obj.StepDuration)));
            %EL
            walkVelo(8,:) = obj.WalkingSpeed* (0.33+  sin(pi+2*pi*t/(obj.StepDuration)));
            %H
            walkVelo(9:end,:) = obj.WalkingSpeed;
            
            
            walkVelo3 = zeros(12,2,size(walkVelo,2));
            for i = 1:size(walkVelo3,1)
              walkVelo3(i,1,:) =  walkVelo(i,:) *cos(DOA(i));
              walkVelo3(i,2,:) =  walkVelo(i,:) *sin(DOA(i));
            end
            
            
            %Find Range and Azimuth Coverage
            edges = zeros(4,2);
            edges(1,:) = [obj.xPos+cosd(obj.heading)*(obj.length)/2, obj.yPos+sind(obj.heading)*(obj.length)/2];
            edges(2,:) = [obj.xPos-cosd(obj.heading)*(obj.length)/2, obj.yPos-sind(obj.heading)*(obj.length)/2];
            edges(3,:) = [obj.xPos+sind(obj.heading)*(obj.width)/2, obj.yPos+cosd(obj.heading)*(obj.width)/2];
            edges(4,:) = [obj.xPos-sind(obj.heading)*(obj.width)/2, obj.yPos-cosd(obj.heading)*(obj.width)/2];
            
            azi = atand(edges(:,2)./edges(:,1));
            
            azi = atand((Contour(:,2)-rPos(2))./(Contour(:,1)-rPos(1)));
            obj.aziCoverage = round(min(azi)):round(max(azi));
            obj.rCoverage = zeros(size(obj.aziCoverage));
            for a = obj.aziCoverage
                searchAngle = 1;
                selectAzi = Contour(azi<a+0.5 & azi>a-0.5,:); %Contour Points with corresponding azi
                while size(selectAzi,1)==0
                    searchAngle = searchAngle+1;
                    selectAzi = Contour(azi<a+0.5*searchAngle & azi>a-0.5*searchAngle,:);
                end
                minR = min(sqrt((selectAzi(:,1)-rPos(1)).^2+(selectAzi(:,2)-rPos(2)).^2)); %find min Contour Point Range for corresponding azi
                if minR < 0 && max(sqrt((selectAzi(:,1)-rPos(1)).^2+(selectAzi(:,2)-rPos(2)).^2)) < 0
                    minR = (1+size(fmcw.rangeBins,2))*fmcw.dR; %Target behind Radar and not visible
                elseif minR < 0 && max(sqrt(selectAzi(:,1).^2+selectAzi(:,2).^2)) > 0
                    minR = fmcw.dR; % Target right in front of Radar, partly behind
                end
                obj.rCoverage(a-obj.aziCoverage(1)+1) = minR;
            end
%             if obj.plotContour
%                 figure
%                 scatter(Contour(:,1),Contour(:,2));
%                 hold on
%             end
            

            
            %--------------------------------------------------------------
            % RCS
            % abs(RCS) for this viewing angle is sampled from measurements
            %
            
            totalRCSdBsm = -6;
            obj.RCSsigma = relRCS* 10^(totalRCSdBsm/10); %(/30 for measurement) in square meters
           
            
            %--------------------------------------------------------------
            % GENERATE POINT TARGETS IN SPECIFIED POSITIONS/VELOCITIES
            % The scattering points of the vehicle's contour and the wheels
            % are collected in a RadarTarget and a Platform moving with
            % the corresponding velocity.
            %
            
            % Set Elevation
            Elref = ones(size(Contour(:,1)))* fmcw.height; %TODO Add height !!!!!!!!!!
            
            
            
            % Collect all Car Scattering Points
            meastime = 0:fmcw.chirpInterval:obj.StepDuration;
            obj.PedestrianTarget = phased.RadarTarget('Model','Nonfluctuating','MeanRCS', obj.RCSsigma.',...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
            
            CustomTrajectory = zeros(length(meastime), 4, size(Contour,1)); % t, x(t), y(t), z(t)
            %CustomTrajectory(:,3,:) = repmat(Contour(:,3)', length(meastime), 1);
            CustomTrajectory(1,2:end,:,:) = Contour.';
            CustomTrajectory(1,1,:,:) = 0;
            for t = 2:length(meastime)
                CustomTrajectory(t,1,:,:) = meastime(t);
                CustomTrajectory(t,2,:) = squeeze(CustomTrajectory(t-1,2,:))+ squeeze(walkVelo3(:,1,t)*fmcw.chirpInterval);
                CustomTrajectory(t,3,:) = squeeze(CustomTrajectory(t-1,3,:))+ squeeze(walkVelo3(:,2,t)*fmcw.chirpInterval);
                CustomTrajectory(t,4,:) = Contour(:,3);
            end  
            obj.TargetPlatform = phased.Platform('InitialPosition',[Contour(:,1)'; Contour(:,2)'; Contour(:,3)'], ...
                    'OrientationAxesOutputPort',true, 'InitialVelocity', [cosd(obj.heading)*obj.WalkingSpeed*ones(size(Contour(:,1)))';...
                    sind(obj.heading)*obj.WalkingSpeed*ones(size(Contour(:,1)))'; ...
                    zeros(size(Contour(:,1)'))], ...
                    'CustomTrajectory', CustomTrajectory,...
                    'MotionModel', 'Custom');
            
        end
        
        
        
        %% Move Target Platform
        function [post,velt,axt, obj] = move(obj, tsamp, ~)
            % Calculates the current target position and velocity after 
            % moving the platform for a duration tsamp.
            tsamp = mod(tsamp, obj.StepDuration);
            [post,velt,axt] = obj.TargetPlatform(tsamp);
        end
        
        
        %% Reflect incoming Signal
        function RXsig = reflect(obj, xtrans, ~)
            % Reflect the signal xtrans from the scattering points of the
            % target object. Angle is not required here, but included to
            % simplify automation.
            if obj.BicyclistTarget.Model == 'Nonfluctuating'
                RXsig = obj.PedestrianTarget(xtrans);                
            else
                RXsig = obj.PedestrianTarget(xtrans, true);
            end
        end
        
        
        %% Release Car Target Object
        function obj = RemoveHiddenScatterers(obj, bool, fmcwc0, fmcwf0)
            % Call radar target release function for changes of reflection
            % points
            RCSsig = obj.RCSsigma;
            RCSsig(bool>0) = [];
            obj.PedestrianTarget = phased.RadarTarget('Model','Nonfluctuating','MeanRCS', RCSsig.',...
                    'PropagationSpeed',fmcwc0,'OperatingFrequency',fmcwf0);
        end
        
        
        %% Restore Car Target Object
        function obj = restoreReflectionPoints(obj, fmcwc0, fmcwf0)
            obj.PedestrianTarget = phased.RadarTarget('Model', 'Nonfluctuating','MeanRCS', obj.RCSsigma.',...
                    'PropagationSpeed',fmcwc0,'OperatingFrequency',fmcwf0);
        end
    end
end

