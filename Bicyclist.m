classdef Bicyclist
    %CLASS BackscatterBicycle class
    %   Generate prominent radar scattering points for a bicycle in certain
    %   position. Consider different bicycle dimensions, moving Wheels (micro 
    %   Doppler), angular positions of target. Neglect Pedaling movements
    %   for now (mostly not radial)
    
    properties
        plotContour = false; %bool: set to true to see scattering points
        
        ID = [];
        typeNr = [];
        xPos = []; %center y position
        yPos = []; %center x position
        heading = []; %angle of movement relative to x-axis
        vel = []; %velocity along heading angle
        width = []; %vehicle width
        length = []; %vehicle length
        Height = []; %vehicle height
        frameHeight = []; %frame height
        rTire = []; %radius of tire
        RCS = []; %RCS real measurement data
        
        ReceptionAngle = 150; % SET MAX INCIDENT ANGLE RANGE FOR RAY RECEPTION [-ReceptionAngle/2, ReceptionAngle/2]
        ReflectionsPerContourPoint = 1; % SET THIS PARAMETER FOR RESOLUTION
        WheelReflectionsFactor = []; % SET TO EMPHASIZE WHEELS
        drefPoints = []; %Number of reflection points
        types = 2; %Number of possible bicycle object types
        
        %for generateObstructionMap()
        aziCoverage = [];  %Azimut coverage
        rCoverage = []; %Range coverage
        leakingRatio = [];
        
        RCSsigma = []; %RCS of individual backscatterers
        InitialHeading = []; %heading in angle relative to x axis
        Acceleration = []; %Acceleration for all move() steps
        N = []; %final Number of Scattering points
        TargetPlatform = []; %target platform
        BicyclistTarget = []; %target object
    end
    
    methods
        %% Initialize Dimensions
        function obj = initBicycle(obj,typeNr)
            %INIT FUNCTION
            %   Load car properties for type specified in typeNr 
            
            obj.ID = ['Bicycle', num2str(typeNr)];
            if typeNr == 0
                %26"
                obj.typeNr = 0; %type of Bike
                obj.width = 0.78; %width of Bike/handlebar
                obj.length = 1.70; %length of Bike
                obj.frameHeight = 0.55; %height of Frame
                obj.Height = 1+rand()*0.3; %height of Bicyclist
                %obj.RCS = [-3, -7, 3,1, -3, 2, 0, 2, 4, 7.5, 5, 6, -2, -2, -3, -4, -3, 1, -7]; % [Geary Motorbike] 
                obj.RCS = [-3, -5, -7, -5, -2, -5, -3, -4, 2, 15, 0, -3, -6, -6, -8, -5, -3, -4, -2];
                obj.rTire = 0.70/2; %radius of a tire
            elseif typeNr == 1
                %29"
                obj.typeNr = 1; %type of vehicle
                obj.width = 0.8; %width of vehicle
                obj.length = 1.80; %length of vehicle
                obj.frameHeight = 0.65; %height of Frame
                obj.Height = 1+rand()*0.6; %height of Bicyclist
                obj.RCS = [-3, -5, -7, -5, -2, -5, -3, -4, 2, 15, 0, -3, -6, -6, -8, -5, -3, -4, -2];
                obj.rTire = 0.78/2; %radius of a tire
            else
                error('Specified bicycle type nr. %i does not exist!',typeNr)
            end
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
        function obj = generateBackscatterTarget(obj,fmcw)
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
            % Define static Reflection Point Locations
            lenFrame = obj.length-obj.rTire*2;
            obj.drefPoints = fmcw.dR; %distance between reference Points >= Radar Resolution
            
            %DOA of radar beam
            azi = atand(obj.yPos./obj.xPos); %azimuth of target Point
            DOA = abs(obj.normAngle(azi-180-obj.heading));

            if abs(DOA)>5 && abs(DOA)<175
                FnumRefPoints = floor(lenFrame/obj.drefPoints); %Frame
                SFnumRefPoints = floor(obj.frameHeight/obj.drefPoints); %Seatpost/Fork
                HnumRefPoints = floor(obj.width/obj.drefPoints); %Handlebar
                PnumRefPoints = 8; %Person rl arms, legs, feet, shoulders
                                
                Contour = zeros(FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+PnumRefPoints,3);
                %Contour = [xRefPoint, yRefPoint, orientationAngle]
                %% Bicycle Frame (side)
                for i = 1:FnumRefPoints
                    % Approx side of Frame with a linear line of reflection
                    % points
                    Contour(i,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2-cosd(obj.heading)*(i-1)*obj.drefPoints; 
                    Contour(i,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2-sind(obj.heading)*(i-1)*obj.drefPoints;
                    Contour(i,3) = obj.normAngle(obj.heading+90); %actually bidirectional angle for frame
                end
                % Consider vertical components
                for i = 1:SFnumRefPoints
                    %Seatpost
                    Contour(i+FnumRefPoints,1) = obj.xPos-cosd(obj.heading)*(0.1*obj.length)-cosd(obj.heading)*0.05/i*obj.length; 
                    Contour(i+FnumRefPoints,2) = obj.yPos-sind(obj.heading)*(0.1*obj.length)-sind(obj.heading)*0.05/i*obj.length;
                    Contour(i+FnumRefPoints,3) = obj.normAngle(obj.heading+90); %actually bidirectional angle for frame
                end
                for i = 1:SFnumRefPoints
                    %Fork
                    Contour(i+FnumRefPoints+SFnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2-cosd(obj.heading)*(rand()*0.8)*obj.rTire+(mod(i,2)-0.5)*0.20*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+SFnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2-sind(obj.heading)*(rand()*0.8)*obj.rTire+(mod(i,2)-0.5)*0.20*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+SFnumRefPoints,3) = obj.normAngle(obj.heading+90); %actually bidirectional angle for frame
                end
                %Handlebar
                for i = 1:HnumRefPoints
                    Contour(i+FnumRefPoints+2*SFnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2-cosd(obj.heading)*0.4*obj.rTire+(rand()-0.5)*obj.width*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+2*SFnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2-sind(obj.heading)*0.4*obj.rTire+(rand()-0.5)*obj.width*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+2*SFnumRefPoints,3) = obj.normAngle(obj.heading); %actually bidirectional angle for frame
                end
                    
                %% Person (side)
                % Arms
                for i = 1:2
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2*(1.2*rand()-0.2)-cosd(obj.heading)*0.4*obj.rTire-(rand())*(-1)^i*obj.width/2*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2*(1.2*rand()-0.2)-sind(obj.heading)*0.4*obj.rTire-(rand())*(-1)^i*obj.width/2*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints,3) = obj.normAngle(obj.heading+(-1)^i*90); %not bidirectional anymore!
                end
                % Shoulders
                for i = 1:2
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+2,1) = obj.xPos+cosd(obj.heading)*(0.25*rand())-(rand())*(-1)^i*obj.width/2*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+2,2) = obj.yPos+sind(obj.heading)*(0.25*rand())-(rand())*(-1)^i*obj.width/2*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+2,3) = obj.normAngle(obj.heading+(-1)^i*90); %not bidirectional anymore!
                end
                % Feet/Legs/Knees
                pedalingSpeed = randn(1,4)*1.5; % max 3.5m/s
                for i = 1:4
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+4,1) = obj.xPos+cosd(obj.heading)*(0.5*rand()-0.25)-(rand())*(-1)^i*obj.width/3*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+4,2) = obj.yPos+sind(obj.heading)*(0.5*rand()-0.25)-(rand())*(-1)^i*obj.width/3*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+2*SFnumRefPoints+HnumRefPoints+4,3) = obj.normAngle(obj.heading+(-1)^i*90); %not bidirectional anymore!
                end
                
            else
                %Front/Back View
                FnumRefPoints = floor(lenFrame/2/obj.drefPoints); %Frame
                SFnumRefPoints = floor(obj.frameHeight/obj.drefPoints); %Seatpost/Fork
                PnumRefPoints = 8;
                if abs(DOA) < 5
                    HnumRefPoints = floor(obj.width/obj.drefPoints); %Handlebar
                    dir = 1;
                else
                    HnumRefPoints = floor(obj.width/obj.drefPoints); %Person
                    dir = -1;
                end
                                
                Contour = zeros(FnumRefPoints+SFnumRefPoints+HnumRefPoints+PnumRefPoints,3);
                %Contour = [xRefPoint, yRefPoint, orientationAngle]
                
                %% Bicycle Frame
                for i = 1:FnumRefPoints
                    % Approx side of Frame with a linear line of reflection
                    % points
                    Contour(i,1) = obj.xPos+dir*cosd(obj.heading)*(lenFrame)/2-dir*cosd(obj.heading)*(i-1)*obj.drefPoints; 
                    Contour(i,2) = obj.yPos+dir*sind(obj.heading)*(lenFrame)/2-dir*sind(obj.heading)*(i-1)*obj.drefPoints;
                    Contour(i,3) = obj.normAngle(obj.heading+(dir-1)/2*180); %actually bidirectional angle for frame
                end
                % Consider vertical components
                if abs(DOA)>175
                    %Back view
                    for i = 1:SFnumRefPoints
                        %Seatpost
                        Contour(i+FnumRefPoints,1) = obj.xPos-cosd(obj.heading)*(0.1*obj.length)-cosd(obj.heading)*0.05/i*obj.length; 
                        Contour(i+FnumRefPoints,2) = obj.yPos-sind(obj.heading)*(0.1*obj.length)-sind(obj.heading)*0.05/i*obj.length;
                        Contour(i+FnumRefPoints,3) = obj.normAngle(obj.heading+90); %actually bidirectional angle for frame
                    end
                    for i = 1:HnumRefPoints
                        %Person
                        Contour(i+FnumRefPoints+SFnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2-cosd(obj.heading)*0.3*obj.rTire+(rand()-0.5)*obj.width*cosd(obj.heading+90); 
                        Contour(i+FnumRefPoints+SFnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2-sind(obj.heading)*0.3*obj.rTire+(rand()-0.5)*obj.width*sind(obj.heading+90);
                        Contour(i+FnumRefPoints+SFnumRefPoints,3) = obj.normAngle(obj.heading+ randn()*20); %actually bidirectional angle for frame
                    end
                    
                else
                    %Front view
                    for i = 1:SFnumRefPoints
                        %Fork
                        Contour(i+FnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2-cosd(obj.heading)*(rand()*0.8)*obj.rTire+(mod(i,2)-0.5)*0.2*cosd(obj.heading+90); 
                        Contour(i+FnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2-sind(obj.heading)*(rand()*0.8)*obj.rTire+(mod(i,2)-0.5)*0.2*sind(obj.heading+90);
                        Contour(i+FnumRefPoints,3) = obj.normAngle(obj.heading+90); %actually bidirectional angle for frame
                    end
                    for i = 1:HnumRefPoints
                        %Handlebar
                        Contour(i+FnumRefPoints+SFnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2-cosd(obj.heading)*0.4*obj.rTire+(rand()-0.5)*obj.width*cosd(obj.heading+90); 
                        Contour(i+FnumRefPoints+SFnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2-sind(obj.heading)*0.4*obj.rTire+(rand()-0.5)*obj.width*sind(obj.heading+90);
                        Contour(i+FnumRefPoints+SFnumRefPoints,3) = obj.normAngle(obj.heading); %actually bidirectional angle for frame
                    end
                end
                
                %% Person
                %% Person (side)
                % Arms
                for i = 1:2
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints,1) = obj.xPos+cosd(obj.heading)*(lenFrame)/2*(1.2*rand()-0.2)-cosd(obj.heading)*0.4*obj.rTire-(rand())*(-1)^i*obj.width/2*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints,2) = obj.yPos+sind(obj.heading)*(lenFrame)/2*(1.2*rand()-0.2)-sind(obj.heading)*0.4*obj.rTire-(rand())*(-1)^i*obj.width/2*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints,3) = obj.normAngle(obj.heading+(-1)^dir*180); %not bidirectional anymore!
                end
                % Shoulders
                for i = 1:2
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints+2,1) = obj.xPos+cosd(obj.heading)*(0.25*rand())-(rand())*(-1)^i*obj.width/2*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints+2,2) = obj.yPos+sind(obj.heading)*(0.25*rand())-(rand())*(-1)^i*obj.width/2*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints+2,3) = obj.normAngle(obj.heading+(-1)^dir*180); %not bidirectional anymore!
                end
                % Feet/Legs/Knees
                pedalingSpeed = randn(1,4)*1.5; % max 3.5m/s
                for i = 1:4
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints+4,1) = obj.xPos+cosd(obj.heading)*(0.5*rand()-0.25)-(rand())*(-1)^i*obj.width/3*cosd(obj.heading+90); 
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints+4,2) = obj.yPos+sind(obj.heading)*(0.5*rand()-0.25)-(rand())*(-1)^i*obj.width/3*sind(obj.heading+90);
                    Contour(i+FnumRefPoints+SFnumRefPoints+HnumRefPoints+4,3) = obj.normAngle(obj.heading+(-1)^dir*180); %not bidirectional anymore!
                end
                
            end
            
            
            
            % ASSIGN RCS VALUE
            
            %Calculate RCS for Contour Points
            azi = atand(Contour(:,2)./Contour(:,1)); %azimuth of target Point
            DOAall = abs(obj.normAngle(azi-180));
            hittingAngle = abs(obj.normAngle(DOAall-Contour(:,3))); %hitting angle at Contour Point
            if sum(hittingAngle(1:end-8) > 90)>0
                sel = [hittingAngle(1:end-8)>90; zeros(8,1)]; % rotate incident angle for bidirectional points, not for person (8)!
                hittingAngle(sel==1) = abs(hittingAngle(sel==1)-180);
            end
            relRCScontour = (1-hittingAngle/180); % const RCS for all reflections from one Contour Point!!!!!!!!!
                       
            
            if obj.plotContour
                figure
                scatter(Contour(1:end-8,1),Contour(1:end-8,2),[], 'b')
                hold on;
                scatter(Contour(end-7:end-4,1),Contour(end-7:end-4,2),[], 'm')
                hold on;
                scatter(Contour(end-3:end,1),Contour(end-3:end,2),[], 'mx')
                hold on;
            end
            
            
            %--------------------------------------------------------------
            % WHEELS
            % Calculate Wheels reflection point positions
            WheelCenter = zeros(2,3);
            %Front
            WheelCenter(1,1) = obj.xPos + cosd(obj.heading)*lenFrame/2;
            WheelCenter(1,2) = obj.yPos + sind(obj.heading)*lenFrame/2;
            WheelCenter(1,3) = obj.normAngle(obj.heading +90);
            %Back
            WheelCenter(2,1) = obj.xPos - cosd(obj.heading)*lenFrame/2;
            WheelCenter(2,2) = obj.yPos - sind(obj.heading)*lenFrame/2;
            WheelCenter(2,3) = obj.normAngle(obj.heading +90);
            
            azi = atand(WheelCenter(:,2)./WheelCenter(:,1)); %azimuth of target Point
            wDOA = obj.normAngle(azi+180); %angle of car relative to radar ray
            
            %Calc max turn rate for Doppler
            turnRate = obj.vel/(2*pi*obj.rTire); % turns per second
            obj.WheelReflectionsFactor = round((4*obj.rTire/obj.drefPoints));
            
            %Check visibility
            if abs(DOA)<5 || abs(DOA)>175
                %Front/Back view
                
                if abs(DOA)<5
                    dir = 1;
                else
                    dir = -1;
                end
                
                % Sample random Positions around Wheel center
                % wheelScatterer = [#wheel, reflections, [xPos,yPos,vel,RCS] ]
                wheelScatterer = zeros(2, obj.WheelReflectionsFactor,4); 
                wheelAcceleration = zeros(2, obj.WheelReflectionsFactor, fmcw.chirpsCycle);
                for i = 1:2 % 2 Wheels
                    %v_wheeel in range vtarget + [-vd, +vd]
                    %Smallest velocity in middle, increasing to the edges
                    %Highest RCS in middle (low v), smaller on the sides (high v)
                    %Area of a Ring with const. v on the wheel: A=pi*(r1^2 - r2^2)
                    % A ~ dA/dr ~ 2dr with dr = r2-r1
                    % => RCS ~ 2r   and   vd ~ r
                    
                    %Half Doppler Model
                    
                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.1; %max diameter of bike tire around 0.2m 
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx* 2*(0.5-rand(obj.WheelReflectionsFactor,1));
                    yi = vary* randn(obj.WheelReflectionsFactor,1)+0.1*dir;
                    yi(abs(yi)>obj.rTire) = vary * 2*(0.5-rand(sum(abs(yi)>obj.rTire),1));
       
                    % Sample local velocity in xi-yi-plane
                    vi = zeros(size(yi));
                    %vi(yi>(obj.rTire)) = 0; % static tire case reflection
                    % tire reflection with rotational velocity
                    %velBins = (-obj.vel:fmcw.dV:obj.vel);
                    vi(abs(yi)<=(obj.rTire)) = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire.* (1-abs(rand(sum(abs(yi)<=(obj.rTire)),1))-0.5)*2;
                    
                    
                    % Acceleration
                    vrot = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire;
                    z = vi(abs(yi)<=(obj.rTire)).*sqrt(obj.rTire.^2 - yi(abs(yi)<=(obj.rTire)).^2)./ (vrot);
                    r = sqrt(z.^2+yi(abs(yi)<=(obj.rTire)).^2);
                    t = acos(vi(abs(yi)<=(obj.rTire))./(vrot)).* r./(vrot);
                    tsamp = t + [0:(fmcw.chirpsCycle-1)] *fmcw.chirpInterval;
                    wheelAcceleration(i,abs(yi)<=(obj.rTire),:) = - (vrot).* sin((vrot)./ r .* tsamp) .* (vrot./r);
                    
                    
                    hittingAngle = abs(obj.normAngle(wDOA(i)-WheelCenter(i,3))); %hitting angle at Contour Point
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = sin(acos(yi(abs(yi)<=obj.rTire)/obj.rTire)); %RCS relative to reflection Position inside tire
                    hiddenFactor = 1;
                    if sum(hittingAngle > 90)>0
                        hittingAngle(hittingAngle>90) = abs(hittingAngle(hittingAngle>90)-180);
                    end
                    relRCS = RCSy.* hiddenFactor; %  (1-2*hittingAngle/180).*
                    
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,:,:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y, obj.vel+vi, relRCS];
                    
                    if obj.plotContour
                        scatter(wheelScatterer(i,:,1),wheelScatterer(i,:,2), [], 'rx')
                        legend('Frame Reflection Points', 'Static Person Reflection Points', 'Dynamic Person Reflection Points', 'Dynamic Tyre Reflection Points')
                        hold on
                    end
                end
                
            else % Angle not betweeen [-5,5] or [175,185]
                
                % Sample random Positions around Wheel center
                % wheelScatterer = [#wheel, reflections, [xPos,yPos,vel,RCS] ]
                wheelScatterer = zeros(2,obj.WheelReflectionsFactor,4); 
                wheelAcceleration = zeros(2, obj.WheelReflectionsFactor, fmcw.chirpsCycle);
                for i = 1:2 % 2 Wheels
                    %v_wheeel in range vtarget + [-vd, +vd]
                    %Smallest velocity in middle, increasing to the sides
                    %Highest RCS in middle (low v), larger on the sides (high v)
                    %Area of a Ring with const. v on the wheel: A=pi*(r1^2 - r2^2)
                    % A ~ dA/dr ~ 2dr with dr = r2-r1
                    % => RCS ~ 2r   and   vd ~ r

                    %Full Doppler Model

                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.1; %max diameter of bike tire around 0.2m 
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx* 2*(0.5-rand(obj.WheelReflectionsFactor,1));
                    yi = vary* randn(obj.WheelReflectionsFactor,1);
                    yi(abs(yi)>obj.rTire) = vary * 2*(0.5-rand(sum(abs(yi)>obj.rTire),1));

                    % Sample local velocity in xi-yi-plane
                    vi = zeros(size(yi));
                    %vi(yi>(obj.rTire)) = 0; % static tire case reflection
                    % tire reflection with rotational velocity
                    %velBins = (-obj.vel:fmcw.dV:obj.vel);
                    vi(abs(yi)<=(obj.rTire)) = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire.* (rand(sum(abs(yi)<=(obj.rTire)),1)-0.5)*2;


                    % Acceleration
                    vrot = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire;
                    z = vi(abs(yi)<=(obj.rTire)).*sqrt(obj.rTire.^2 - yi(abs(yi)<=(obj.rTire)).^2)./ (vrot);
                    r = sqrt(z.^2+yi(abs(yi)<=(obj.rTire)).^2);
                    t = acos(vi(abs(yi)<=(obj.rTire))./(vrot)).* r./(vrot);
                    tsamp = t + [0:(fmcw.chirpsCycle-1)] *fmcw.chirpInterval;
                    wheelAcceleration(i,abs(yi)<=(obj.rTire),:) = - (vrot).* sin((vrot)./ r .* tsamp) .* (vrot./r);

                    hittingAngle = abs(obj.normAngle(wDOA(i)-WheelCenter(i,3))); %hitting angle at Contour Point
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = sin(acos(yi(abs(yi)<=obj.rTire)/obj.rTire)); %RCS relative to reflection Position inside tire
                    hiddenFactor = 1;
                    if hittingAngle > 90
                        hittingAngle(hittingAngle>90) = abs(hittingAngle(hittingAngle>90)-180);
                    end
                    relRCS = RCSy.* hiddenFactor; % (1-2*(hittingAngle)/180).*

                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);

                    wheelScatterer(i,:,:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y, obj.vel+vi, relRCS];

                    if obj.plotContour
                        scatter(wheelScatterer(i,:,1),wheelScatterer(i,:,2), [], 'rx')
                        legend('Frame/Person Reflection Points', 'Dynamic Tyre Reflection Points')
                        hold on
                    end
                end
            end
            if obj.plotContour
                title('Reflection Points sampled for Bicyclist')
                xlabel('x Position (m)')
                ylabel('y Position (m)')
            end
            
             % Filter low RCS points
            wheelScatterer = reshape(wheelScatterer, [], 4); %allign Scatterers in a Column Vector
            wheelAcceleration = reshape(wheelAcceleration, [], fmcw.chirpsCycle);
            empty = wheelScatterer(:,1)==0 & wheelScatterer(:,2) == 0 & wheelScatterer(:,3) == 0;
            if sum(empty)>0
                fprintf('There were uninitialized Scatterers\n')
            end
            wheelScatterer(empty,:) = []; %filter empty Scatterers from hidden wheels
            wheelAcceleration(empty,:) = []; 
            
            
            
            
            Scatterer = [Contour(:,1:2), relRCScontour; wheelScatterer(:,1:2), wheelScatterer(:,4)];
            
            %Find Range and Azimuth Coverage
            edges = zeros(4,2);
            edges(1,:) = [obj.xPos+cosd(obj.heading)*(obj.length)/2, obj.yPos+sind(obj.heading)*(obj.length)/2];
            edges(2,:) = [obj.xPos-cosd(obj.heading)*(obj.length)/2, obj.yPos-sind(obj.heading)*(obj.length)/2];
            edges(3,:) = [obj.xPos+sind(obj.heading)*(obj.width)/2, obj.yPos+cosd(obj.heading)*(obj.width)/2];
            edges(4,:) = [obj.xPos-sind(obj.heading)*(obj.width)/2, obj.yPos-cosd(obj.heading)*(obj.width)/2];
            
            azi = atand(edges(:,2)./edges(:,1));
            
            azi = atand(Scatterer(:,2)./Scatterer(:,1));
            obj.aziCoverage = round(min(azi)):round(max(azi));
            obj.rCoverage = zeros(size(obj.aziCoverage));
            for a = obj.aziCoverage
                searchAngle = 1;
                selectAzi = Scatterer(azi<a+0.5 & azi>a-0.5,:); %Scatterer Points with corresponding azi
                while size(selectAzi,1)==0
                    searchAngle = searchAngle+1;
                    selectAzi = Scatterer(azi<a+0.5*searchAngle & azi>a-0.5*searchAngle,:);
                end
                minR = min(sqrt(selectAzi(:,1).^2+selectAzi(:,2).^2)); %find min Contour Point Range for corresponding azi
                if minR < 0 && max(sqrt(selectAzi(:,1).^2+selectAzi(:,2).^2)) < 0
                    minR = (1+size(fmcw.rangeBins,2))*fmcw.dR; %Target behind Radar and not visible
                elseif minR < 0 && max(sqrt(selectAzi(:,1).^2+selectAzi(:,2).^2)) > 0
                    minR = fmcw.dR; % Target right in front of Radar, partly behind
                end
                obj.rCoverage(a-obj.aziCoverage(1)+1) = minR;
            end
%             if obj.plotContour
%                 figure
%                 scatter(Scatterer(:,1),Scatterer(:,2));
%                 hold on
%             end
            
            
            
            
            
            
            
            %--------------------------------------------------------------
            % BODY & UNDERBODY REFLECTIONS
            % Sample gaussian reflections in center of vehicle
            
            
            

            
            %--------------------------------------------------------------
            % RCS
            % abs(RCS) for this viewing angle is sampled from measurements
            %
            
            
            hittingAngle = abs(normAngle(obj,DOA));
            RCSdBsm = interp1(0:10:180, obj.RCS, hittingAngle); % Find RCS for corresponding DOA from measurement data
            relRCS = 1/sum(Scatterer(:,3)) * Scatterer(:,3)'; % relative RCS contribution [0,1]
            obj.RCSsigma = relRCS* 10^(RCSdBsm/10); %(/30 for measurement) in square meters
            
            
            %--------------------------------------------------------------
            % GENERATE POINT TARGETS IN SPECIFIED POSITIONS/VELOCITIES
            % The scattering points of the vehicle's contour and the wheels
            % are collected in a RadarTarget and a Platform moving with
            % the corresponding velocity.
            %
            
            % Set Elevation
            Elref = ones(size(Scatterer(:,1)))* fmcw.height; %TODO Add height !!!!!!!!!!
            
            
            
            % Collect all Car Scattering Points
            % Swerling2 optional
            obj.BicyclistTarget = phased.RadarTarget('Model','Nonfluctuating','MeanRCS', obj.RCSsigma,...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
                    
            obj.TargetPlatform = phased.Platform('InitialPosition',[Scatterer(:,1)'; Scatterer(:,2)'; Elref'], ...
                    'OrientationAxesOutputPort',true, 'InitialVelocity', [cosd(obj.heading)*obj.vel*ones(size(Contour(1:end-4,1)))', ...
                    cosd(obj.heading)*pedalingSpeed, cosd(obj.heading).*wheelScatterer(:,3)';...
                    sind(obj.heading)*obj.vel*ones(size(Contour(1:end-4,1)))', sind(obj.heading)*pedalingSpeed, sind(obj.heading).*wheelScatterer(:,3)'; ...
                    zeros(size(Contour(:,1)')), zeros(size(wheelScatterer(:,1)'))], ...
                    'Acceleration', [zeros(size(Contour(:,3)')), cosd(obj.heading).*wheelAcceleration(:,1)';...
                    zeros(size(Contour(:,3)')), sind(obj.heading).*wheelAcceleration(:,1)'; ...
                    zeros(size(Contour(:,1)')), zeros(size(wheelScatterer(:,1)'))], ...
                    'MotionModel', 'Acceleration', 'AccelerationSource', 'Input port');
            
            % Parameters for Wheel Point Acceleration (TargetPlatform)
            obj.Acceleration = wheelAcceleration;
            obj.N = size(Scatterer(:,1),1); %Num of reflector Points in Car Target
        end
        
        
        
        %% Move Target Platform
        function [post,velt,axt, obj] = move(obj, tsamp, ~)
            % Calculates the current target position and velocity after 
            % moving the platform for a duration tsamp.
            lenContour = obj.N-size(obj.Acceleration(:,1),1);
            tstep = 1;
            A = [zeros(1,lenContour), cosd(obj.heading).*obj.Acceleration(:,tstep)';...
                    zeros(1,lenContour), sind(obj.heading).*obj.Acceleration(:,tstep)'; ...
                    zeros(1,lenContour), zeros(size(obj.Acceleration(:,tstep)'))];
            [post,velt,axt] = obj.TargetPlatform(tsamp, A);
            obj.Acceleration(:,tstep) = [];
%             scatter(post(1,:),post(2,:))
%             hold on;
%             pause(0.01);
        end
        
        
        %% Reflect incoming Signal
        function RXsig = reflect(obj, xtrans, ~)
            % Reflect the signal xtrans from the scattering points of the
            % target object. Angle is not required here, but included to
            % simplify automation.
            RXsig = obj.BicyclistTarget(xtrans, true);
        end
        
        
        %% Release Car Target Object
        function obj = RemoveHiddenScatterers(obj, bool, fmcwc0, fmcwf0)
            % Call radar target release function for changes of reflection
            % points
            RCSsig = obj.RCSsigma;
            RCSsig(bool>0) = [];
            obj.BicyclistTarget = phased.RadarTarget('Model','Swerling2','MeanRCS', RCSsig,...
                    'PropagationSpeed',fmcwc0,'OperatingFrequency',fmcwf0);
        end
        
        
        %% Restore Car Target Object
        function obj = restoreReflectionPoints(obj, fmcwc0, fmcwf0)
            obj.BicyclistTarget = phased.RadarTarget('Model', 'Swerling2','MeanRCS', obj.RCSsigma,...
                    'PropagationSpeed',fmcwc0,'OperatingFrequency',fmcwf0);
        end
    end
end

