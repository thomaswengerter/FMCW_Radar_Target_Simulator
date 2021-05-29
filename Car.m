classdef Car
    %CLASS BackscatterVehicle class
    %   Generate prominent radar scattering points for a car in certain
    %   position. Consider different vehicle shapes, moving Wheels (micro 
    %   Doppler), angular positions of target.
    
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
        heightAxis = []; %Height of wheelbase/axis above ground
        cornerRadius = []; %curve radius of front to sides 
        rTire = []; %radius of tire
        RCS = []; %RCS real measurement data
        
        
        ReceptionAngle = 160; % SET MAX INCIDENT ANGLE RANGE FOR RAY RECEPTION [-ReceptionAngle/2, ReceptionAngle/2]
        ReflectionsPerContourPoint = 1; % SET THIS PARAMETER FOR RESOLUTION
        WheelReflectionsFactor = 4; % SET TO EMPHASIZE WHEELS
        drefPoints = []; %Number of reflection points
        types = 2; %Number of possible car object types
        
        %for generateObstructionMap()
        aziCoverage = [];  %Azimut coverage
        rCoverage = []; %Range coverage
        
        RCSsigma = []; %RCS of individual backscatterers
        InitialHeading = []; %only Spaceholder for move() function call!
        Acceleration = []; %Acceleration for all move() steps
        N = []; %final Number of Scattering points
        TargetPlatform = []; %target platform
        CarTarget = []; %target object
    end
    
    methods
        %% Initialize Dimensions
        function obj = initCar(obj,typeNr)
            %INIT FUNCTION
            %   Load car properties for type specified in typeNr 
            
            obj.ID = ['Vehicle', num2str(typeNr)];
            if typeNr == 0
                %Standard Car Dimensions
                obj.typeNr = 0; %type of vehicle
                obj.width = 1.8; %width of vehicle
                obj.length = 4.5; %length of vehicle
                obj.Height = 1.5; %height of vehicle
                obj.heightAxis = 0.3; %distance from car underbody to ground
                obj.cornerRadius = 0.8; %radius of contour corners
                obj.RCS = [10, 7, 6, 5, 4, 4, 3, 2, 5, 20, 5, 0, 0, 0, 1, 3, 5, 10, 15]; %measured RCS in dBsm azi= [0:180]
                %obj.RCS = mean([-1, 4,-3,-8,-12,-2, -5,-2,-6,-4,+7,+5, -2,-5,-4,-11,-9,+3, -3; ...
                           %fliplr([-3,-7,-12,-8,-6,-4, -12,-5,-7,+1,-1,+5, -9,-10,-5,-2,-5,0, -2])], 1); % [Abadpour] 
                obj.rTire = 0.3; %radius of a tire
            elseif typeNr == 1
                %Jeep/Transporter
                obj.typeNr = 1; %type of vehicle
                obj.width = 2.01; %width of vehicle
                obj.length = 5.8; %length of vehicle
                obj.Height = 2.1; %height of vehicle
                obj.heightAxis = 0.375; %distance from car underbody to ground
                obj.cornerRadius = 0.7; %radius of contour corners
                obj.RCS = 4+ [10, 7, 6, 5, 4, 4, 3, 2, 5, 20, 5, 0, 0, 0, 1, 3, 5, 10, 15]; %measured RCS in dBsm azi= [0:180]
                %obj.RCS = 4+ mean([-1, 4,-3,-8,-12,-2, -5,-2,-6,-4,+7,+5, -2,-5,-4,-11,-9,+3, -3; ...
                           %fliplr([-3,-7,-12,-8,-6,-4, -12,-5,-7,+1,-1,+5, -9,-10,-5,-2,-5,0, -2])], 1); % [Abadpour] 
                obj.rTire = 0.375; %radius of a tire
            else
                error('Specified car type Nr. %i does not exist!',typeNr)
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
        function obj = generateBackscatterTarget(obj,fmcw,rPos)
            %   For current measurement setup, generate the Point Modell
            %   with corresponding RCS. Consider visability, surface
            %   propagation, 
            
            
%             %Some Parameters
%             radarPos = [0,0,fmcw.height]; %static Radar position
%             azi = atand(obj.yPos/obj.xPos); %azimuth of target
%             R = sqrt(obj.xPos^2+obj.yPos^2); %range of target
            
            
            %--------------------------------------------------------------
            % CONTOUR 
            % Calculate Contour Reflection Point Locations
            
            obj.drefPoints = fmcw.dR; %distance between reference Points >= Radar Resolution
            WnumRefPoints = floor(obj.width/obj.drefPoints);
            LnumRefPoints = floor(obj.length/obj.drefPoints);
            
            Contour = zeros(2*(WnumRefPoints+LnumRefPoints), 3);
            %Contour = [xRefPoint, yRefPoint, orientationAngle]
            wtangent = obj.heading+90; %angle in deg
            if mod(WnumRefPoints,2)==1 %uneven #Points
                %Front of Vehicle
                Contour(1,1) = obj.xPos+cosd(obj.heading)*obj.length/2; 
                Contour(1,2) = obj.yPos+sind(obj.heading)*obj.length/2;
                Contour(1,3) = obj.normAngle(obj.heading);
                %Back of Vehicle
                Contour(1+WnumRefPoints+LnumRefPoints,1) = obj.xPos-cosd(obj.heading)*obj.length/2;
                Contour(1+WnumRefPoints+LnumRefPoints,2) = obj.yPos-sind(obj.heading)*obj.length/2;
                Contour(1+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+180);
                for i = 2:2:WnumRefPoints
                    %Front of Vehicle, middle to sides
                    Contour(i,1) = Contour(1,1)+i/2*obj.drefPoints*cosd(wtangent);
                    Contour(i,2) = Contour(1,2)+i/2*obj.drefPoints*sind(wtangent);
                    Contour(i,3) = obj.normAngle(obj.heading); %TODO: change heading at corners and max -180,180!!!!!!!!
                    Contour(i+1,1) = Contour(1,1)-i/2*obj.drefPoints*cosd(wtangent); 
                    Contour(i+1,2) = Contour(1,2)-i/2*obj.drefPoints*sind(wtangent);
                    Contour(i+1,3) = obj.normAngle(obj.heading);
                    distCorner = obj.width/2-i/2*obj.drefPoints; %dist to corner point
                    if distCorner<=obj.cornerRadius
                        %Flatten front corners with x^2 contour approximation
                        wctangent = wtangent+ (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i,1) = Contour(i-2,1)+obj.drefPoints*cosd(wctangent);
                        Contour(i,2) = Contour(i-2,2)+obj.drefPoints*sind(wctangent);
                        Contour(i,3) = obj.normAngle(obj.heading+(1-distCorner/obj.cornerRadius)* 45);
                        wctangent = wtangent-180-(1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+1,1) = Contour(i-1,1)+obj.drefPoints*cosd(wctangent); 
                        Contour(i+1,2) = Contour(i-1,2)+obj.drefPoints*sind(wctangent);
                        Contour(i+1,3) = obj.normAngle(obj.heading-(1-distCorner/obj.cornerRadius)* 45);
                    end
                    
                    %Back of Vehicle, middle to sides
                    Contour(i+WnumRefPoints+LnumRefPoints,1) = Contour(1+WnumRefPoints+LnumRefPoints,1)+i/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,2) = Contour(1+WnumRefPoints+LnumRefPoints,2)+i/2*obj.drefPoints*sind(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,1) = Contour(1+WnumRefPoints+LnumRefPoints,1)-i/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,2) = Contour(1+WnumRefPoints+LnumRefPoints,2)-i/2*obj.drefPoints*sind(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                    if distCorner<=obj.cornerRadius/2
                        %Flatten back corners with x^2 contour approximation
                        wctangent = wtangent+180 - (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+WnumRefPoints+LnumRefPoints,1) = Contour(i+WnumRefPoints+LnumRefPoints-2,1)-obj.drefPoints*cosd(wctangent);
                        Contour(i+WnumRefPoints+LnumRefPoints,2) = Contour(i+WnumRefPoints+LnumRefPoints-2,2)-obj.drefPoints*sind(wctangent);
                        Contour(i+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+180-(1-distCorner/obj.cornerRadius)* 45);
                        wctangent = wtangent+180-180 +(1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+WnumRefPoints+LnumRefPoints+1,1) = Contour(i+WnumRefPoints+LnumRefPoints-1,1)-obj.drefPoints*cosd(wctangent); 
                        Contour(i+WnumRefPoints+LnumRefPoints+1,2) = Contour(i+WnumRefPoints+LnumRefPoints-1,2)-obj.drefPoints*sind(wctangent);
                        Contour(i+WnumRefPoints+LnumRefPoints+1,3) = obj.normAngle(obj.heading+180+(1-distCorner/obj.cornerRadius)* 45);
                    end
                end
                
            elseif mod(WnumRefPoints,2)==0 %even #Points
                %Front of Vehicle
                Contour(1,1) = obj.xPos+cosd(obj.heading)*obj.length/2+0.5*obj.drefPoints*cosd(wtangent); 
                Contour(1,2) = obj.yPos+sind(obj.heading)*obj.length/2+0.5*obj.drefPoints*sind(wtangent);
                Contour(1,3) = obj.normAngle(obj.heading);
                Contour(2,1) = obj.xPos+cosd(obj.heading)*obj.length/2-0.5*obj.drefPoints*cosd(wtangent); 
                Contour(2,2) = obj.yPos+sind(obj.heading)*obj.length/2-0.5*obj.drefPoints*sind(wtangent);
                Contour(2,3) = obj.normAngle(obj.heading);
                %Back of Vehicle
                Contour(1+WnumRefPoints+LnumRefPoints,1) = obj.xPos-cosd(obj.heading)*obj.length/2+0.5*obj.drefPoints*cosd(wtangent);
                Contour(1+WnumRefPoints+LnumRefPoints,2) = obj.yPos-sind(obj.heading)*obj.length/2+0.5*obj.drefPoints*sind(wtangent);
                Contour(1+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                Contour(2+WnumRefPoints+LnumRefPoints,1) = obj.xPos-cosd(obj.heading)*obj.length/2-0.5*obj.drefPoints*cosd(wtangent);
                Contour(2+WnumRefPoints+LnumRefPoints,2) = obj.yPos-sind(obj.heading)*obj.length/2-0.5*obj.drefPoints*sind(wtangent);
                Contour(2+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                
                for i = 3:2:WnumRefPoints
                    %Front of Vehicle, middle to sides
                    Contour(i,1) = Contour(1,1)+(i-1)/2*obj.drefPoints*cosd(wtangent);
                    Contour(i,2) = Contour(1,2)+(i-1)/2*obj.drefPoints*sind(wtangent);
                    Contour(i,3) = obj.normAngle(obj.heading);
                    Contour(i+1,1) = Contour(2,1)-(i-1)/2*obj.drefPoints*cosd(wtangent); 
                    Contour(i+1,2) = Contour(2,2)-(i-1)/2*obj.drefPoints*sind(wtangent);
                    Contour(i+1,3) = obj.normAngle(obj.heading);
                    distCorner = obj.width/2-(i-1)/2*obj.drefPoints; %dist to corner point
                    if distCorner<=obj.cornerRadius
                        %Flatten front corners with x^2 contour approximation
                        wctangent = wtangent + (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i,1) = Contour(i-2,1)+obj.drefPoints*cosd(wctangent);
                        Contour(i,2) = Contour(i-2,2)+obj.drefPoints*sind(wctangent);
                        Contour(i,3) = obj.normAngle(obj.heading+(1-distCorner/obj.cornerRadius)* 45);
                        wctangent = wtangent-180-(1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+1,1) = Contour(i-1,1)+obj.drefPoints*cosd(wctangent); 
                        Contour(i+1,2) = Contour(i-1,2)+obj.drefPoints*sind(wctangent);
                        Contour(i+1,3) = obj.normAngle(obj.heading-(1-distCorner/obj.cornerRadius)* 45);
                    end
                    
                    
                    %Back of Vehicle, middle to sides
                    Contour(i+WnumRefPoints+LnumRefPoints,1) = Contour(1+WnumRefPoints+LnumRefPoints,1)+(i-1)/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,2) = Contour(1+WnumRefPoints+LnumRefPoints,2)+(i-1)/2*obj.drefPoints*sind(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,1) = Contour(2+WnumRefPoints+LnumRefPoints,1)-(i-1)/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,2) = Contour(2+WnumRefPoints+LnumRefPoints,2)-(i-1)/2*obj.drefPoints*sind(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                    if distCorner<=obj.cornerRadius/2
                        %Flatten back corners with x^2 contour approximation
                        wctangent = wtangent+180 -(1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+WnumRefPoints+LnumRefPoints,1) = Contour(i+WnumRefPoints+LnumRefPoints-2,1)-obj.drefPoints*cosd(wctangent);
                        Contour(i+WnumRefPoints+LnumRefPoints,2) = Contour(i+WnumRefPoints+LnumRefPoints-2,2)-obj.drefPoints*sind(wctangent);
                        Contour(i+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+180-(1-distCorner/obj.cornerRadius)* 45);
                        wctangent = wtangent+180-180 +(1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+WnumRefPoints+LnumRefPoints+1,1) = Contour(i+WnumRefPoints+LnumRefPoints-1,1)-obj.drefPoints*cosd(wctangent); 
                        Contour(i+WnumRefPoints+LnumRefPoints+1,2) = Contour(i+WnumRefPoints+LnumRefPoints-1,2)-obj.drefPoints*sind(wctangent);
                        Contour(i+WnumRefPoints+LnumRefPoints+1,3) = obj.normAngle(obj.heading+180+(1-distCorner/obj.cornerRadius)* 45);
                    end
                end
            end
                
                
            ltangent = obj.heading; %angle in deg
            if mod(LnumRefPoints,2)==1 %uneven #Points
                %Right side of Vehicle
                Contour(WnumRefPoints+1,1) = obj.xPos+sind(ltangent)*obj.width/2; 
                Contour(WnumRefPoints+1,2) = obj.yPos-cosd(ltangent)*obj.width/2;
                Contour(WnumRefPoints+1,3) = obj.normAngle(obj.heading-90);
                %Left side of Vehicle
                Contour(1+2*WnumRefPoints+LnumRefPoints,1) = obj.xPos-sind(ltangent)*obj.width/2;
                Contour(1+2*WnumRefPoints+LnumRefPoints,2) = obj.yPos+cosd(ltangent)*obj.width/2;
                Contour(1+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                for i = 2:2:LnumRefPoints
                    %Right side of Vehicle, middle to sides
                    Contour(WnumRefPoints+i,1) = Contour(WnumRefPoints+1,1)+i/2*obj.drefPoints*cosd(ltangent);
                    Contour(WnumRefPoints+i,2) = Contour(WnumRefPoints+1,2)+i/2*obj.drefPoints*sind(ltangent);
                    Contour(WnumRefPoints+i,3) = obj.normAngle(obj.heading-90);
                    Contour(WnumRefPoints+i+1,1) = Contour(WnumRefPoints+1,1)-i/2*obj.drefPoints*cosd(ltangent); 
                    Contour(WnumRefPoints+i+1,2) = Contour(WnumRefPoints+1,2)-i/2*obj.drefPoints*sind(ltangent);
                    Contour(WnumRefPoints+i+1,3) = obj.normAngle(obj.heading-90);
                    %Left side of Vehicle, middle to sides
                    Contour(i+2*WnumRefPoints+LnumRefPoints,1) = Contour(1+2*WnumRefPoints+LnumRefPoints,1)+i/2*obj.drefPoints*cosd(ltangent);
                    Contour(i+2*WnumRefPoints+LnumRefPoints,2) = Contour(1+2*WnumRefPoints+LnumRefPoints,2)+i/2*obj.drefPoints*sind(ltangent);
                    Contour(i+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                    Contour(i+1+2*WnumRefPoints+LnumRefPoints,1) = Contour(1+2*WnumRefPoints+LnumRefPoints,1)-i/2*obj.drefPoints*cosd(ltangent);
                    Contour(i+1+2*WnumRefPoints+LnumRefPoints,2) = Contour(1+2*WnumRefPoints+LnumRefPoints,2)-i/2*obj.drefPoints*sind(ltangent);
                    Contour(i+1+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                    distCorner = obj.length/2-i/2*obj.drefPoints; %dist to corner point
                    if distCorner<=obj.cornerRadius
                        %Right Front
                        lctangent = ltangent+ (1-distCorner/obj.cornerRadius)* 45;
                        Contour(WnumRefPoints+i,1) = Contour(i+WnumRefPoints-2,1)+obj.drefPoints*cosd(lctangent);
                        Contour(WnumRefPoints+i,2) = Contour(i+WnumRefPoints-2,2)+obj.drefPoints*sind(lctangent);
                        Contour(WnumRefPoints+i,3) = obj.normAngle(obj.heading-90+(1-distCorner/obj.cornerRadius)* 45); %change heading at corners
                        %Left Front
                        lctangent = ltangent- (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+2*WnumRefPoints+LnumRefPoints,1) = Contour(i+2*WnumRefPoints+LnumRefPoints-1,1)+obj.drefPoints*cosd(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,2) = Contour(i+2*WnumRefPoints+LnumRefPoints-1,2)+obj.drefPoints*sind(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90+(1-distCorner/obj.cornerRadius)* 45);
                    end
                    % Smaller Back Corners
                    if distCorner<=obj.cornerRadius/2
                        lctangent = ltangent-180- (1-distCorner/obj.cornerRadius)* 45;
                        %Right back corner
                        Contour(WnumRefPoints+i+1,1) = Contour(WnumRefPoints+i-1,1)+obj.drefPoints*cosd(lctangent);
                        Contour(WnumRefPoints+i+1,2) = Contour(WnumRefPoints+i-1,2)+obj.drefPoints*sind(lctangent);
                        Contour(WnumRefPoints+i+1,3) = obj.normAngle(obj.heading-90- (1-distCorner/obj.cornerRadius)* 45); %change heading at corners
                        %Left back corner
                        lctangent = ltangent-180+ (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+2*WnumRefPoints+LnumRefPoints+1,1) = Contour(i-1+2*WnumRefPoints+LnumRefPoints,1)+obj.drefPoints*cosd(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints+1,2) = Contour(i-1+2*WnumRefPoints+LnumRefPoints,2)+obj.drefPoints*sind(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints+1,3) = obj.normAngle(obj.heading+90+ (1-distCorner/obj.cornerRadius)* 45);
                    end
                end
                
            elseif mod(LnumRefPoints,2)==0 %even #Points
                %Right Side of Vehicle
                Contour(WnumRefPoints+1,1) = obj.xPos+sind(ltangent)*obj.width/2+0.5*obj.drefPoints*cosd(ltangent); 
                Contour(WnumRefPoints+1,2) = obj.yPos-cosd(ltangent)*obj.width/2+0.5*obj.drefPoints*sind(ltangent);
                Contour(WnumRefPoints+1,3) = obj.normAngle(obj.heading-90);
                Contour(WnumRefPoints+2,1) = obj.xPos+sind(ltangent)*obj.width/2-0.5*obj.drefPoints*cosd(ltangent); 
                Contour(WnumRefPoints+2,2) = obj.yPos-cosd(ltangent)*obj.width/2-0.5*obj.drefPoints*sind(ltangent);
                Contour(WnumRefPoints+2,3) = obj.normAngle(obj.heading-90);
                %Left Side of Vehicle
                Contour(1+2*WnumRefPoints+LnumRefPoints,1) = obj.xPos-sind(ltangent)*obj.width/2+0.5*obj.drefPoints*cosd(ltangent);
                Contour(1+2*WnumRefPoints+LnumRefPoints,2) = obj.yPos+cosd(ltangent)*obj.width/2+0.5*obj.drefPoints*sind(ltangent);
                Contour(1+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                Contour(2+2*WnumRefPoints+LnumRefPoints,1) = obj.xPos-sind(ltangent)*obj.width/2-0.5*obj.drefPoints*cosd(ltangent);
                Contour(2+2*WnumRefPoints+LnumRefPoints,2) = obj.yPos+cosd(ltangent)*obj.width/2-0.5*obj.drefPoints*sind(ltangent);
                Contour(2+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                
                for i = 3:2:LnumRefPoints
                    %Right side of Vehicle, middle to sides
                    Contour(WnumRefPoints+i,1) = Contour(WnumRefPoints+1,1)+(i-1)/2*obj.drefPoints*cosd(ltangent);
                    Contour(WnumRefPoints+i,2) = Contour(WnumRefPoints+1,2)+(i-1)/2*obj.drefPoints*sind(ltangent);
                    Contour(WnumRefPoints+i,3) = obj.normAngle(obj.heading-90);
                    Contour(WnumRefPoints+i+1,1) = Contour(WnumRefPoints+2,1)-(i-1)/2*obj.drefPoints*cosd(ltangent); 
                    Contour(WnumRefPoints+i+1,2) = Contour(WnumRefPoints+2,2)-(i-1)/2*obj.drefPoints*sind(ltangent);
                    Contour(WnumRefPoints+i+1,3) = obj.normAngle(obj.heading-90);
                    %Left side of Vehicle, middle to sides
                    Contour(i+2*WnumRefPoints+LnumRefPoints,1) = Contour(1+2*WnumRefPoints+LnumRefPoints,1)+(i-1)/2*obj.drefPoints*cosd(ltangent);
                    Contour(i+2*WnumRefPoints+LnumRefPoints,2) = Contour(1+2*WnumRefPoints+LnumRefPoints,2)+(i-1)/2*obj.drefPoints*sind(ltangent);
                    Contour(i+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                    Contour(i+1+2*WnumRefPoints+LnumRefPoints,1) = Contour(2+2*WnumRefPoints+LnumRefPoints,1)-(i-1)/2*obj.drefPoints*cosd(ltangent);
                    Contour(i+1+2*WnumRefPoints+LnumRefPoints,2) = Contour(2+2*WnumRefPoints+LnumRefPoints,2)-(i-1)/2*obj.drefPoints*sind(ltangent);
                    Contour(i+1+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90);
                    distCorner = obj.length/2-(i-1)/2*obj.drefPoints; %dist to corner point
                    if distCorner<=obj.cornerRadius
                        lctangent = ltangent+(1-distCorner/obj.cornerRadius)* 45;
                        %Right front
                        Contour(WnumRefPoints+i,1) = Contour(WnumRefPoints+i-2,1)+obj.drefPoints*cosd(lctangent);
                        Contour(WnumRefPoints+i,2) = Contour(WnumRefPoints+i-2,2)+obj.drefPoints*sind(lctangent);
                        Contour(WnumRefPoints+i,3) = obj.normAngle(obj.heading-90+ (1-distCorner/obj.cornerRadius)* 45); %change heading at corners
                        %Left front
                        lctangent = ltangent- (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+2*WnumRefPoints+LnumRefPoints,1) = Contour(i-2+2*WnumRefPoints+LnumRefPoints,1)+obj.drefPoints*cosd(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,2) = Contour(i-2+2*WnumRefPoints+LnumRefPoints,2)+obj.drefPoints*sind(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90- (1-distCorner/obj.cornerRadius)* 45);
                    end
                    % Smaller Back Corners
                    if distCorner<=obj.cornerRadius/2
                        lctangent = ltangent-180- (1-distCorner/obj.cornerRadius)* 45;
                        %Right back corner
                        Contour(WnumRefPoints+i+1,1) = Contour(WnumRefPoints+i-1,1)+obj.drefPoints*cosd(lctangent);
                        Contour(WnumRefPoints+i+1,2) = Contour(WnumRefPoints+i-1,2)+obj.drefPoints*sind(lctangent);
                        Contour(WnumRefPoints+i+1,3) = obj.normAngle(obj.heading-90- (1-distCorner/obj.cornerRadius)* 45); %change heading at corners
                        %Left back corner
                        lctangent = ltangent-180+ (1-distCorner/obj.cornerRadius)* 45;
                        Contour(i+2*WnumRefPoints+LnumRefPoints+1,1) = Contour(i-1+2*WnumRefPoints+LnumRefPoints,1)+obj.drefPoints*cosd(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints+1,2) = Contour(i-1+2*WnumRefPoints+LnumRefPoints,2)+obj.drefPoints*sind(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints+1,3) = obj.normAngle(obj.heading+90+ (1-distCorner/obj.cornerRadius)* 45);
                    end
                    
                end
                
            end
            fullContour = Contour;
            
            %Find Range and Azimuth Coverage
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
                elseif minR < 0 && max(sqrt((selectAzi(:,1)-rPos(1)).^2+(selectAzi(:,2)-rPos(2)).^2)) > 0
                    minR = fmcw.dR; % Target right in front of Radar, partly behind
                end
                obj.rCoverage(a-obj.aziCoverage(1)+1) = minR; 
            end
            if obj.plotContour
                figure
                scatter(Contour(:,1),Contour(:,2));
                hold on
            end
            
            
            %--------------------------------------------------------------
            % FILTER
            % Remove hidden Contour Points from model
            % Check if Contour Point is in the radar's Field of View:
            % If relative DOA is larger than the specif scattering Point's
            % ReceptionAngle, discard the hidden Contour Point
            
            azi = atand(Contour(:,2)./Contour(:,1)); %azimuth of target Point
            DOA = obj.normAngle(azi+180); %angle of car relative to radar ray 
            filter = abs(obj.normAngle(DOA-Contour(:,3))) > obj.ReceptionAngle/2;
            Contour(filter,:) = [];
            DOA(filter) = [];
            
            if obj.plotContour
                scatter(Contour(:,1),Contour(:,2),[], 'r')
                hold on;
            end
            
            
            
            %--------------------------------------------------------------
            % SAMPLE STATIC RADAR TARGETS
            % Contour scatter Points with constant bulk velocity
            %
            % Impose probability distributions across x/y dimensions
            % x in [0,rangeBins(end)]                   with Resolution dR
            % y in [-rangeBins(end), rangeBins(end)]    with Resolution dR
            % 
            
            ReflectionsPerPoint = obj.ReflectionsPerContourPoint;
            
            %Scatterer = [xPos, yPos, vel, RCS]
            Scatterer = zeros(ReflectionsPerPoint*size(Contour,1), 4);
            
            %Sample Contour Reflections
            for i = 1:size(Contour,1)
                % Coordinate transform to x~ and y~
                
                % Spread wave over ~10 wavelengths
                varx = 0.15; %max around 0.15m from surface
                vary = obj.drefPoints/2; % ~Contour Point distance (~dR)
                meany = -sind(DOA(i)-Contour(i,3))*0.15; % mean dep. on DOA
                
                % Sample Reflection Locations for Contour Point
                yloc = meany + vary * randn(ReflectionsPerPoint,1);
                xloc = varx * raylrnd(ones(ReflectionsPerPoint,1));
                
                % Coordinate Transformation
                [x, y] = toLocal(obj,xloc,yloc, obj.normAngle(Contour(i,3)-180));
                
                %Correct Reflections outside the car contours
                overshoot = (Contour(i,1)+x)>max(fullContour(:,1)) | (Contour(i,1)+x)<min(fullContour(:,1)) | ...
                        (Contour(i,2)+y)>max(fullContour(:,2)) | (Contour(i,2)+y)<min(fullContour(:,2));
                yloc(overshoot) = 0;
                xloc(overshoot) = 0.15;
                [x,y] = toLocal(obj,xloc,yloc, obj.normAngle(Contour(i,3)-180));
                
                %Calculate RCS for this
                hittingAngle = abs(obj.normAngle(DOA(i)-Contour(i,3))); %hitting angle at Contour Point
                relRCS = (1-2*hittingAngle/obj.ReceptionAngle); % const RCS for all reflections from one Contour Point!!!!!!!!!
                
                %Add to List
                Scatterer((i-1)*ReflectionsPerPoint+1:i*ReflectionsPerPoint,:) = [Contour(i,1)+x, Contour(i,2)+y, obj.vel*ones(size(x)), relRCS*ones(size(x))];
            end
            
            if obj.plotContour
                scatter(Scatterer(:,1),Scatterer(:,2),[], 'g.')
                hold on
            end
            
            
            %--------------------------------------------------------------
            % WHEELS
            % Calculate Wheels reflection point positions
            WheelCenter = zeros(4,3);
            %FrontLeft
            WheelCenter(1,1) = obj.xPos + cosd(obj.heading)*obj.length/3 + cosd(obj.heading+90)*obj.width/2;
            WheelCenter(1,2) = obj.yPos + sind(obj.heading)*obj.length/3 + sind(obj.heading+90)*obj.width/2;
            WheelCenter(1,3) = obj.normAngle(obj.heading +90);
            %FrontRight
            WheelCenter(2,1) = obj.xPos + cosd(obj.heading)*obj.length/3 - cosd(obj.heading+90)*obj.width/2;
            WheelCenter(2,2) = obj.yPos + sind(obj.heading)*obj.length/3 - sind(obj.heading+90)*obj.width/2;
            WheelCenter(2,3) = obj.normAngle(obj.heading -90);
            %BackLeft
            WheelCenter(3,1) = obj.xPos - cosd(obj.heading)*obj.length/3 + cosd(obj.heading+90)*obj.width/2;
            WheelCenter(3,2) = obj.yPos - sind(obj.heading)*obj.length/3 + sind(obj.heading+90)*obj.width/2;
            WheelCenter(3,3) = obj.normAngle(obj.heading +90);
            %BackRight
            WheelCenter(4,1) = obj.xPos - cosd(obj.heading)*obj.length/3 - cosd(obj.heading+90)*obj.width/2;
            WheelCenter(4,2) = obj.yPos - sind(obj.heading)*obj.length/3 - sind(obj.heading+90)*obj.width/2;
            WheelCenter(4,3) = obj.normAngle(obj.heading -90);
            
            %Calc max turn rate for Doppler
            turnRate = obj.vel/(2*pi*obj.rTire); % turns per second
            
            
            %Check visibility
            azi = atand(WheelCenter(:,2)./WheelCenter(:,1)); %azimuth of target Point
            wDOA = obj.normAngle(azi+180); %angle of car relative to radar ray 
            hidden = zeros(1,4);
            hidden(abs(normAngle(obj, wDOA - WheelCenter(:,3)))>95) = 1; %Behind visible Contour
            hidden(abs(normAngle(obj, wDOA- WheelCenter(:,3)))<=95 & ...
                    abs(normAngle(obj, wDOA-WheelCenter(:,3)))>=85) = 2; %Front or Back View -> Special Case
    
                
            % Sample random Positions around Wheel center
            % wheelScatterer = [#wheel, reflections, [xPos,yPos,vel,RCS] ]
            wheelScatterer = zeros(4,ReflectionsPerPoint*obj.WheelReflectionsFactor,4); 
            wheelAcceleration = zeros(4, ReflectionsPerPoint*obj.WheelReflectionsFactor, fmcw.chirpsCycle);
            for i = 1:4 % 4 Wheels
                %v_wheeel in range vtarget + [-vd, +vd]
                %Smallest velocity in middle, increasing to the sides
                %Smallest RCS in middle (low v), larger on the sides (high v)
                %Area of a Ring with const. v on the wheel: A=pi*(r1^2 - r2^2)
                % A ~ dA/dr ~ 2dr with dr = r2-r1
                % => RCS ~ 2r   and   vd ~ r
                
                if hidden(i) == 0
                    %Full Doppler Model
                    
                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.15*1.5; %max around inside 0.3m vehicle body
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx* raylrnd(ones(ReflectionsPerPoint*obj.WheelReflectionsFactor,1));
                    yi = vary* randn(ReflectionsPerPoint*obj.WheelReflectionsFactor,1);
       
                    % Sample local velocity in xi-yi-plane
                    vi = zeros(size(yi));
                    vi(abs(yi)>(obj.rTire)) = 0; % static tire case reflection
                    % tire reflection with rotational velocity
                    %velBins = (-obj.vel:fmcw.dV:obj.vel);
                    vi(abs(yi)<=(obj.rTire)) = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire.* (rand(sum(abs(yi)<=(obj.rTire)),1)-0.5)*2;
                    
                    
                    % Acceleration
                    vrad = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire;
                    z = vi(abs(yi)<=(obj.rTire)).*sqrt(obj.rTire.^2 - yi(abs(yi)<=(obj.rTire)).^2)./ (vrad);
                    r = sqrt(z.^2+yi(abs(yi)<=(obj.rTire)).^2);
                    t = acos(vi(abs(yi)<=(obj.rTire))./(vrad)).* r./(vrad);
                    if ~isempty(t)
                        tsamp = t + [0:(fmcw.chirpsCycle-1)] *fmcw.chirpInterval;
                        wheelAcceleration(i,abs(yi)<=(obj.rTire),:) = - (vrad).* sin((vrad)./ r .* tsamp) .* (vrad./r);
                    end
                        
                    hittingAngle = abs(obj.normAngle(wDOA(i)-WheelCenter(i,3))); %hitting angle at Contour Point
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = sin(acos(yi(abs(yi)<=obj.rTire)/obj.rTire)); %RCS relative to reflection Position inside tire
                    hiddenFactor = 1;
                    relRCS = 0.4 * RCSy.* hiddenFactor;
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,:,:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y, obj.vel+vi, relRCS];
                    
                    if obj.plotContour
                        scatter(wheelScatterer(i,:,1),wheelScatterer(i,:,2), [], 'y.')
                        hold on
                    end
                        
                elseif hidden(i) == 1
                    % Side view, hidden Wheels: Only lower part visible with
                    %relative Speed [-vel,0]
                    
                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.15; %max around inside 0.3m vehicle body
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx* raylrnd(ones(floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),1));
                    yi = vary* randn(floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),1);
                    
                    % Sample local velocity in xi-yi-plane
                    vi = zeros(size(yi));
                    vi(abs(yi)>(obj.rTire)) = 0; % static tire case reflection
                    % tire reflection with rotational velocity
                    %velBins = (-obj.vel:fmcw.dV:0);
                    vi(abs(yi)<=(obj.rTire)) = -obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire.* rand(sum(abs(yi)<=(obj.rTire)),1);

                    % Acceleration
                    vrad = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire;
                    z = vi(abs(yi)<=(obj.rTire)).*sqrt(obj.rTire.^2 - yi(abs(yi)<=(obj.rTire)).^2)./ (vrad);
                    r = sqrt(z.^2+yi(abs(yi)<=(obj.rTire)).^2);
                    t = acos(vi(abs(yi)<=(obj.rTire))./(vrad)).* r./(vrad);
                    if ~isempty(t)
                        tsamp = t + [0:(fmcw.chirpsCycle-1)] *fmcw.chirpInterval;
                        wheelAcceleration(i,abs(yi)<=(obj.rTire),:) = - (vrad).* sin((vrad)./ r .* tsamp) .* (vrad./r);
                    end
                    
                    hittingAngle = abs(obj.normAngle(wDOA(i)-180-WheelCenter(i,3))); %hitting angle at back of wheel
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = (obj.rTire-abs(yi(abs(yi)<obj.rTire)))/obj.rTire; %RCS relative to reflection Position inside tire
                    hiddenFactor = obj.heightAxis/fmcw.height; %should be ~0.5
                    %ADD FACTOR FOR REFLECTIONS UNDER CAR
                    relRCS = 0.3 *RCSy.* hiddenFactor;
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,1:floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y,  obj.vel+vi, relRCS];
                    
                    if obj.plotContour
                        scatter(wheelScatterer(i,1:floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),1),wheelScatterer(i,1:floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),2), [], 'm.')
                        hold on
                    end
                    
                elseif hidden(i) == 2
                    % Front/Back view of tire
                    % relative Speed [-vel,0]
                    
                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.15; %max around inside 0.3m vehicle body
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx+ varx/2* randn(floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),1);
                    yi = vary* randn(floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),1);
                    % Sample local velocity in xi-yi-plane
                    vi = zeros(size(yi));
                    vi(yi<(obj.rTire)) = 0; % static tire case reflection
                    % tire reflection with rotational velocity
                    %velBins = (-obj.vel:fmcw.dV:0);
                    vi(abs(yi)<=(obj.rTire)) = -obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire.* rand(sum(abs(yi)<=obj.rTire),1);
                    
                    % Acceleration
                    vrad = obj.vel.* (obj.rTire-abs(yi(abs(yi)<=(obj.rTire))))./obj.rTire;
                    z = vi(abs(yi)<=(obj.rTire)).*sqrt(obj.rTire.^2 - yi(abs(yi)<=(obj.rTire)).^2)./ (vrad);
                    r = sqrt(z.^2+yi(abs(yi)<=(obj.rTire)).^2);
                    t = acos(vi(abs(yi)<=(obj.rTire))./(vrad)).* r./(vrad);
                    if ~isempty(t)
                        tsamp = t + [0:(fmcw.chirpsCycle-1)] *fmcw.chirpInterval;
                        wheelAcceleration(i,abs(yi)<=(obj.rTire),:) = - (vrad).* sin((vrad)./ r .* tsamp) .* (vrad./r);
                    end
                    
                    hittingAngle = abs(mod(obj.normAngle(wDOA(i)-WheelCenter(i,3)),180)); %hitting angle in front/back of wheel
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = (obj.rTire-abs(yi(abs(yi)<obj.rTire)))/obj.rTire; %RCS relative to reflection Position inside tire
                    hiddenFactor = obj.heightAxis/fmcw.height;
                    % DIFFER BETWEEN BACK/FRONT AND ADD ATT FACTOR FOR
                    % REFLECTIONS UNDER VEHICLE
                    if hittingAngle > 90
                        hittingAngle = abs(hittingAngle-180);
                    end
                    relRCS = 0.1* RCSy.* hiddenFactor;
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,1:floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y, obj.vel+vi, relRCS];
                    
                    if obj.plotContour
                        scatter(wheelScatterer(i,1:floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),1),wheelScatterer(i,1:floor(ReflectionsPerPoint*obj.WheelReflectionsFactor/2),2), [], 'mx')
                        hold on
                    end
                end
            end
            
             % Filter low RCS points
            wheelScatterer = reshape(wheelScatterer, [], 4); %allign Scatterers in a Column Vector
            wheelAcceleration = reshape(wheelAcceleration, [], fmcw.chirpsCycle);
            empty = wheelScatterer(:,1)==0 & wheelScatterer(:,2) == 0 & wheelScatterer(:,3) == 0;
            wheelScatterer(empty,:) = []; %filter empty Scatterers from hidden wheels
            wheelAcceleration(empty,:) = []; 
            
            %--------------------------------------------------------------
            % BODY & UNDERBODY REFLECTIONS
            % Sample gaussian reflections in center of vehicle
            
            
            

            
            %--------------------------------------------------------------
            % RCS
            % abs(RCS) for this viewing angle is sampled from measurements
            %
            
            azi = atand(obj.yPos./obj.xPos); %azimuth of target Point
            DOA = abs(obj.normAngle(azi-180-obj.heading));
            hittingAngle = abs(normAngle(obj,DOA));
            RCSdBsm = interp1(0:10:180, obj.RCS, hittingAngle); % Find RCS for corresponding DOA from measurement data
            relRCS = 1/sum([Scatterer(:,4)', wheelScatterer(:,4)']) * [Scatterer(:,4)', wheelScatterer(:,4)']; % relative RCS contribution [0,1]
            obj.RCSsigma = relRCS* 10^(RCSdBsm/10); %(/30 for measurement) in square meters
            
            
            %--------------------------------------------------------------
            % GENERATE POINT TARGETS IN SPECIFIED POSITIONS/VELOCITIES
            % The scattering points of the vehicle's contour and the wheels
            % are collected in a RadarTarget and a Platform moving with
            % the corresponding velocity.
            %
            
            % Set Elevation
            Elref = ones(size(Scatterer(:,1)))* fmcw.height; %TODO Add height !!!!!!!!!!
            wDOA = obj.normAngle(atand(wheelScatterer(:,2)./wheelScatterer(:,1))+180);
            WheelElref = rand(size(wDOA))* obj.rTire*2; %TODO Add height !!!!!!!!!!
            
            
            % Collect all Car Scattering Points
            obj.CarTarget = phased.RadarTarget('Model','Nonfluctuating','MeanRCS', obj.RCSsigma,...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
            
            obj.TargetPlatform = phased.Platform('InitialPosition',[Scatterer(:,1)', wheelScatterer(:,1)'; Scatterer(:,2)',wheelScatterer(:,2)'; Elref', WheelElref'], ...
                    'OrientationAxesOutputPort',true, 'InitialVelocity', [cosd(obj.heading)*Scatterer(:,3)', cosd(obj.heading).*wheelScatterer(:,3)';...
                    sind(obj.heading)*Scatterer(:,3)', sind(obj.heading).*wheelScatterer(:,3)'; ...
                    zeros(size(Scatterer(:,1)')), zeros(size(wheelScatterer(:,1)'))], ...
                    'Acceleration', [zeros(size(Scatterer(:,3)')), cosd(obj.heading).*wheelAcceleration(:,1)';...
                    zeros(size(Scatterer(:,3)')), sind(obj.heading).*wheelAcceleration(:,1)'; ...
                    zeros(size(Scatterer(:,1)')), zeros(size(wheelScatterer(:,1)'))], ...
                    'MotionModel', 'Acceleration', 'AccelerationSource', 'Input port');
            
            % Parameters for Wheel Point Acceleration (TargetPlatform)
            obj.Acceleration = wheelAcceleration;
            obj.N = size(Scatterer(:,1),1)+size(wheelScatterer(:,1),1); %Num of reflector Points in Car Target
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
            if strcmp(obj.CarTarget.Model, 'Nonfluctuating')
                RXsig = obj.CarTarget(xtrans);
            else
            	RXsig = obj.CarTarget(xtrans, true); %update RCS with Swerling2
            end
        end
        
        
        
        %% Release Car Target Object
        function obj = RemoveHiddenScatterers(obj, bool, fmcwc0, fmcwf0)
            % Call radar target release function for changes of reflection
            % points
            RCSsig = obj.RCSsigma;
            RCSsig(bool>0) = [];
            obj.CarTarget = phased.RadarTarget('Model','Nonfluctuating','MeanRCS', RCSsig,...
                    'PropagationSpeed',fmcwc0,'OperatingFrequency',fmcwf0);
        end
        
        
        %% Restore Car Target Object
        function obj = restoreReflectionPoints(obj, fmcwc0, fmcwf0)
            obj.CarTarget = phased.RadarTarget('Model', 'Nonfluctuating','MeanRCS', obj.RCSsigma,...
                    'PropagationSpeed',fmcwc0,'OperatingFrequency',fmcwf0);
        end
    end
end

