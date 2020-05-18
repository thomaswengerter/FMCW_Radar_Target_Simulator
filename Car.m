classdef Car
    %CLASS BackscatterVehicle class
    %   Generate prominent radar scattering points for a car in certain
    %   position. Consider different vehicle shapes, moving Wheels (micro 
    %   Doppler), angular positions of target.
    
    properties
        ID = [];
        xPos = []; %center y position
        yPos = []; %center x position
        heading = []; %angle of movement relative to x-axis
        vel = []; %velocity along heading angle
        width = []; %vehicle width
        length = []; %vehicle length
        heightAxis = []; %Height of wheelbase/axis above ground
        cornerRadius = [];
        rTire = [];
        
        
        drefPoints = []; %Number of reflection points
        types = 2; %Number of possible car object types
        
        
        RCS = [];
        InitialHeading = [];
        TargetPlatform = [];
        CarTarget = [];
    end
    
    methods
        %% Initialize Dimensions
        function obj = initCar(obj,typeNr)
            %INIT FUNCTION
            %   Load car properties for type specified in typeNr 
            
            obj.ID = ['Vehicle', num2str(typeNr)];
            if typeNr == 0
                %Standard Car Dimensions
                obj.width = 1.8;
                obj.length = 4.5;
                obj.heightAxis = 0.8;
                obj.cornerRadius = 0.8; %radius of contour corners
                obj.RCS = load('RCS_AUDI'); %estimated max RCS in Square Meters azi= [0:180]
                obj.rTire = 0.3; %radius of a tire
            elseif typeNr == 1
                %TODO: add random cars, SUVs, trucks, ...
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
        function obj = generateBackscatterTarget(obj,fmcw)
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
            
            figure
            scatter(Contour(:,1),Contour(:,2));
            hold on
            
            
            
            %--------------------------------------------------------------
            % FILTER
            % Remove hidden Contour Points from model
            % Check if Contour Point is in the radar's Field of View:
            % If relative DOA is larger than the specif scattering Point's
            % ReceptionAngle, discard the hidden Contour Point
            
            ReceptionAngle = 120; % SET RECEPTION ANGLE APPROPRIATELY
            % ReceptionAngle 90° -> [-45°,45°] 
            
            azi = atand(Contour(:,2)./Contour(:,1)); %azimuth of target Point
            DOA = obj.normAngle(azi+180); %angle of car relative to radar ray 
            filter = abs(obj.normAngle(DOA-Contour(:,3))) > ReceptionAngle/2;
            Contour(filter,:) = [];
            DOA(filter) = [];
            
            scatter(Contour(:,1),Contour(:,2),[], 'r')
            hold on;
            
            
            
            
            %--------------------------------------------------------------
            % SAMPLE STATIC RADAR TARGETS
            % Contour scatter Points with constant bulk velocity
            %
            % Impose probability distributions across x/y dimensions
            % x in [0,rangeBins(end)]                   with Resolution dR
            % y in [-rangeBins(end), rangeBins(end)]    with Resolution dR
            % 
            
            ReflectionsPerPoint = 1;
            
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
                relRCS = (1-2*hittingAngle/ReceptionAngle); % const RCS for all reflections from one Contour Point!!!!!!!!!
                
                %Add to List
                Scatterer((i-1)*ReflectionsPerPoint+1:i*ReflectionsPerPoint,:) = [Contour(i,1)+x, Contour(i,2)+y, obj.vel*ones(size(x)), relRCS*ones(size(x))];
            end
            
            scatter(Scatterer(:,1),Scatterer(:,2),[], 'g.')
            hold on
            
            
            
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
            hidden(abs(normAngle(obj, wDOA - WheelCenter(:,3)))>100) = 1; %Behind visible Contour
            hidden(abs(normAngle(obj, wDOA- WheelCenter(:,3)))<=100 & ...
                    abs(normAngle(obj, wDOA-WheelCenter(:,3)))>=80) = 2; %Front or Back View -> Special Case
    
                
            % Sample random Positions around Wheel center
            % wheelScatterer = [#wheel, reflections, [xPos,yPos,vel,RCS] ]
            WheelReflectionsFactor = 5;
            wheelScatterer = zeros(4,ReflectionsPerPoint*WheelReflectionsFactor,4); 
            
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
                    xi = varx* raylrnd(ones(ReflectionsPerPoint*WheelReflectionsFactor,1));
                    yi = vary* randn(ReflectionsPerPoint*WheelReflectionsFactor,1);
                    % Sample local velocity in xi-yi-plane
                    if yi>(obj.rTire)
                        vi = 0; % static tire case reflection
                    else
                        % tire reflection with rotational velocity
                        %velBins = (-obj.vel:fmcw.dV:obj.vel);
                        vi = obj.vel.* (obj.rTire-abs(yi))./obj.rTire.* (rand(ReflectionsPerPoint*WheelReflectionsFactor,1)-0.5)*2;
                    end
                    
                    hittingAngle = abs(obj.normAngle(wDOA(i)-WheelCenter(i,3))); %hitting angle at Contour Point
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = (obj.rTire-abs(yi(abs(yi)<obj.rTire)))/obj.rTire; %RCS relative to reflection Position inside tire
                    hiddenFactor = 1;
                    relRCS = (1-2*hittingAngle/180).* RCSy.* hiddenFactor;
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,:,:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y, obj.vel+vi, relRCS];
                    
                    scatter(wheelScatterer(i,:,1),wheelScatterer(i,:,2), [], 'y.')
                    hold on
                    
                elseif hidden(i) == 1
                    % Side view, hidden Wheels: Only lower part visible with
                    %relative Speed [-vel,0]
                    
                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.15; %max around inside 0.3m vehicle body
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx* raylrnd(ones(floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1));
                    yi = vary* randn(floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1);
                    % Sample local velocity in xi-yi-plane
                    if yi>(obj.rTire)
                        vi = 0; % static tire case reflection
                    else
                        % tire reflection with rotational velocity
                        %velBins = (-obj.vel:fmcw.dV:0);
                        vi = -obj.vel.* (obj.rTire-abs(yi))./obj.rTire.* rand(floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1);
                    end
                    
                    hittingAngle = abs(obj.normAngle(wDOA(i)-180-WheelCenter(i,3))); %hitting angle at back of wheel
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = (obj.rTire-abs(yi(abs(yi)<obj.rTire)))/obj.rTire; %RCS relative to reflection Position inside tire
                    hiddenFactor = obj.heightAxis/fmcw.height;
                    %ADD FACTOR FOR REFLECTIONS UNDER CAR
                    relRCS = (1-2*hittingAngle/180) .* RCSy.* hiddenFactor;
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,1:floor(ReflectionsPerPoint*WheelReflectionsFactor/2),:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y,  obj.vel+vi, relRCS];
                    
                    scatter(wheelScatterer(i,1:floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1),wheelScatterer(i,1:floor(ReflectionsPerPoint*WheelReflectionsFactor/2),2), [], 'm.')
                    hold on
                    
                elseif hidden(i) == 2
                    % Front/Back view of tire
                    % relative Speed [-vel,0]
                    
                    % Coordinate Transform in Wheel Center: xi, yi
                    varx = 0.15; %max around inside 0.3m vehicle body
                    vary = obj.rTire; % ~radius of tire 
                    xi = varx+ varx/2* randn(floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1);
                    yi = vary* randn(floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1);
                    % Sample local velocity in xi-yi-plane
                    if yi>(obj.rTire)
                        vi = 0; % static tire case reflection
                    else
                        % tire reflection with rotational velocity
                        %velBins = (-obj.vel:fmcw.dV:0);
                        vi = -obj.vel.* (obj.rTire-abs(yi))./obj.rTire.* rand(floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1);
                    end
                    
                    hittingAngle = abs(mod(obj.normAngle(wDOA(i)-WheelCenter(i,3)),180)); %hitting angle in front/back of wheel
                    RCSy = ones(size(yi));
                    RCSy(abs(yi)<obj.rTire) = (obj.rTire-abs(yi(abs(yi)<obj.rTire)))/obj.rTire; %RCS relative to reflection Position inside tire
                    hiddenFactor = obj.heightAxis/fmcw.height;
                    % DIFFER BETWEEN BACK/FRONT AND ADD ATT FACTOR FOR
                    % REFLECTIONS UNDER VEHICLE
                    relRCS = (1-2*hittingAngle/180).* RCSy.* hiddenFactor;
                    
                    %Trafo back to original Coordinate System
                    [x,y] = toLocal(obj, xi, yi, WheelCenter(i,3)-180);
                    
                    wheelScatterer(i,1:floor(ReflectionsPerPoint*WheelReflectionsFactor/2),:) = [WheelCenter(i,1)+x, WheelCenter(i,2)+y, obj.vel+vi, relRCS];
                    
                    scatter(wheelScatterer(i,1:floor(ReflectionsPerPoint*WheelReflectionsFactor/2),1),wheelScatterer(i,1:floor(ReflectionsPerPoint*WheelReflectionsFactor/2),2), [], 'mx')
                    hold on
                end
            end
            
             % Filter low RCS points
            wheelScatterer = reshape(wheelScatterer, [], 4); %allign Scatterers in a Column Vector
            wheelScatterer(wheelScatterer(:,1)==0 & wheelScatterer(:,2) == 0 & wheelScatterer(:,3) == 0,:) = []; %filter empty Scatterers from hidden wheels
            
            
            %--------------------------------------------------------------
            % BODY & UNDERBODY REFLECTIONS
            % Sample gaussian reflections in center of vehicle
            
            
            

            
            %--------------------------------------------------------------
            % SAMPLE RCS FROM MEASUREMENTS
            % abs(RCS) for this viewing angle is sampled from measurements
            %
            azi = atand(obj.yPos./obj.xPos); %azimuth of target Point
            DOA = abs(obj.normAngle(azi-180-obj.heading));
            RCSm2 = obj.RCS.rcs(floor(abs(normAngle(obj,DOA))+1)); % Find RCS for corresponding DOA from measurement data
            relRCS = 1/sum([Scatterer(:,4)', wheelScatterer(:,4)']) * [Scatterer(:,4)', wheelScatterer(:,4)']; % relative RCS contribution [0,1]
            RCSsigma = relRCS* RCSm2/100; %in square meters
            
            
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
            obj.CarTarget = phased.RadarTarget('Model','Swerling2','MeanRCS', RCSsigma,...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
            
            obj.TargetPlatform = phased.Platform('InitialPosition',[Scatterer(:,1)', wheelScatterer(:,1)'; Scatterer(:,2)',wheelScatterer(:,2)'; Elref', WheelElref'], ...
                    'OrientationAxesOutputPort',true, 'Velocity', [cosd(obj.heading)*Scatterer(:,3)', cosd(obj.heading).*wheelScatterer(:,3)';...
                    sind(obj.heading)*Scatterer(:,3)', sind(obj.heading).*wheelScatterer(:,3)'; ...
                    zeros(size(Scatterer(:,1)')), zeros(size(wheelScatterer(:,1)'))], 'Acceleration', [0;0;0]);
            
        end
        
        
        
        %% Move Target Platform
        function [post,velt,axt] = move(obj, tsamp, ~)
            % Calculates the current target position and velocity after 
            % moving the platform for a duration tsamp.
            [post,velt,axt] = obj.TargetPlatform(tsamp);
        end
        
        
        %% Reflect incoming Signal
        function RXsig = reflect(obj, xtrans, angle)
            % Reflect the signal xtrans from the scattering points of the
            % target object. Angle is not required here, but included to
            % simplify automation.
            RXsig = obj.CarTarget(xtrans, true);
        end
        
    end
end

