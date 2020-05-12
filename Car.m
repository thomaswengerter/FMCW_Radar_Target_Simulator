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
        surfaceAtt = []; %Factor for surface wave attenuation density
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
                obj.heightAxis = 0.3;
                obj.surfaceAtt = exp(log(0.5)/5e-6); % half-value layer d1/2 = 5e-6 m
                obj.cornerRadius = 0.3; %radius of contour corners
                obj.RCS = 1.5; %estimated max RCS in Square Meters
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
        
        
        %% Generate Point Modell with RCS
        function obj = generateBackscatterTarget(obj,fmcw)
            %   For current measurement setup, generate the Point Modell
            %   with corresponding RCS. Consider visability, surface
            %   propagation, 
            
            radarPos = [0,0,fmcw.height]; %static Radar position
            azi = atand(obj.yPos/obj.xPos); %azimuth of target
            R = sqrt(obj.xPos^2+obj.yPos^2); %range of target
            
            
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
                    distCorner = obj.width/2-(i-1)/2*obj.drefPoints; %dist to corner point
                    if distCorner<=obj.cornerRadius
                        %Flatten front corners with x^2 contour approximation
                        wctangent = wtangent+atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        Contour(i,1) = Contour(1,1)+i/2*obj.drefPoints*cosd(wtangent)+obj.drefPoints*cosd(wctangent);
                        Contour(i,2) = Contour(1,2)+i/2*obj.drefPoints*sind(wtangent)+obj.drefPoints*sind(wctangent);
                        Contour(i,3) = obj.normAngle(obj.heading+atand((obj.cornerRadius-dc)/obj.cornerRadius));
                        wctangent = wtangent-180-atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        Contour(i+1,1) = Contour(1,1)-i/2*obj.drefPoints*cosd(wtangent)+obj.drefPoints*cosd(wctangent); 
                        Contour(i+1,2) = Contour(1,2)-i/2*obj.drefPoints*sind(wtangent)+obj.drefPoints*sind(wctangent);
                        Contour(i+1,3) = obj.normAngle(obj.heading-atand((obj.cornerRadius-dc)/obj.cornerRadius));
                    end
                    %Back of Vehicle, middle to sides
                    Contour(i+WnumRefPoints+LnumRefPoints,1) = Contour(1+WnumRefPoints+LnumRefPoints,1)+i/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,2) = Contour(1+WnumRefPoints+LnumRefPoints,2)+i/2*obj.drefPoints*sind(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,1) = Contour(1+WnumRefPoints+LnumRefPoints,1)-i/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,2) = Contour(1+WnumRefPoints+LnumRefPoints,2)-i/2*obj.drefPoints*sind(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
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
                        wctangent = wtangent+atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        Contour(i,1) = Contour(1,1)+(i-3)/2*obj.drefPoints*cosd(wtangent)+obj.drefPoints*cosd(wctangent);
                        Contour(i,2) = Contour(1,2)+(i-3)/2*obj.drefPoints*sind(wtangent)+obj.drefPoints*sind(wctangent);
                        Contour(i,3) = obj.normAngle(obj.heading+atand((obj.cornerRadius-distCorner)/obj.cornerRadius));
                        wctangent = wtangent-180-(atand((obj.cornerRadius-distCorner)/obj.cornerRadius));
                        Contour(i+1,1) = Contour(2,1)-(i-3)/2*obj.drefPoints*cosd(wtangent)+obj.drefPoints*cosd(wctangent); 
                        Contour(i+1,2) = Contour(2,2)-(i-3)/2*obj.drefPoints*sind(wtangent)+obj.drefPoints*sind(wctangent);
                        Contour(i+1,3) = obj.normAngle(obj.heading-atand((obj.cornerRadius-distCorner)/obj.cornerRadius));
                    end
                    %Back of Vehicle, middle to sides
                    Contour(i+WnumRefPoints+LnumRefPoints,1) = Contour(1+WnumRefPoints+LnumRefPoints,1)+(i-1)/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,2) = Contour(1+WnumRefPoints+LnumRefPoints,2)+(i-1)/2*obj.drefPoints*sind(wtangent);
                    Contour(i+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,1) = Contour(2+WnumRefPoints+LnumRefPoints,1)-(i-1)/2*obj.drefPoints*cosd(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,2) = Contour(2+WnumRefPoints+LnumRefPoints,2)-(i-1)/2*obj.drefPoints*sind(wtangent);
                    Contour(i+1+WnumRefPoints+LnumRefPoints,3) = obj.normAngle(180+obj.heading);
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
                        lctangent = ltangent+atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        Contour(WnumRefPoints+i,1) = Contour(WnumRefPoints+1,1)+(i-2)/2*obj.drefPoints*cosd(ltangent)+obj.drefPoints*cosd(lctangent);
                        Contour(WnumRefPoints+i,2) = Contour(WnumRefPoints+1,2)+(i-2)/2*obj.drefPoints*sind(ltangent)+obj.drefPoints*sind(lctangent);
                        Contour(WnumRefPoints+i,3) = obj.normAngle(obj.heading-90+atand((obj.cornerRadius-distCorner)/obj.cornerRadius)); %change heading at corners
                        %Left Front
                        lctangent = ltangent-atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,1) = Contour(1+2*WnumRefPoints+LnumRefPoints,1)+(i-2)/2*obj.drefPoints*cosd(ltangent)+obj.drefPoints*cosd(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,2) = Contour(1+2*WnumRefPoints+LnumRefPoints,2)+(i-2)/2*obj.drefPoints*sind(ltangent)+obj.drefPoints*sind(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90+atand((obj.cornerRadius-distCorner)/obj.cornerRadius));
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
                        lctangent = ltangent+atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        %Right front
                        Contour(WnumRefPoints+i,1) = Contour(WnumRefPoints+1,1)+(i-3)/2*obj.drefPoints*cosd(ltangent)+obj.drefPoints*cosd(lctangent);
                        Contour(WnumRefPoints+i,2) = Contour(WnumRefPoints+1,2)+(i-3)/2*obj.drefPoints*sind(ltangent)+obj.drefPoints*sind(lctangent);
                        Contour(WnumRefPoints+i,3) = obj.normAngle(obj.heading-90+atand((obj.cornerRadius-distCorner)/obj.cornerRadius)); %change heading at corners
                        %Left front
                        lctangent = ltangent-atand((obj.cornerRadius-distCorner)/obj.cornerRadius);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,1) = Contour(1+2*WnumRefPoints+LnumRefPoints,1)+(i-3)/2*obj.drefPoints*cosd(ltangent)+obj.drefPoints*cosd(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,2) = Contour(1+2*WnumRefPoints+LnumRefPoints,2)+(i-3)/2*obj.drefPoints*sind(ltangent)+obj.drefPoints*sind(lctangent);
                        Contour(i+2*WnumRefPoints+LnumRefPoints,3) = obj.normAngle(obj.heading+90-atand((obj.cornerRadius-distCorner)/obj.cornerRadius));
                    end
                end
                
            end
            figure
            scatter(Contour(:,1),Contour(:,2));
            hold on
            
            % Filter relevant Points in Field of View
            DOA = obj.normAngle(obj.heading+azi+180); %angle of car relative to radar ray 
            %DOAtarget = normAngle(obj, DOA - 180);
            ReceptionAngle = 90;
            Contour(abs(obj.normAngle(Contour(:,3)-DOA)) > ReceptionAngle/2,:) = [];
            
            
            %--------------------------------------------------------------
            % WHEELS
            % Calculate Wheels reflection point positions
            WheelCenter = zeros(4,2);
            %FrontLeft
            WheelCenter(1,1) = obj.xPos + cosd(obj.heading)*obj.length/3 + cosd(obj.heading+90)*obj.width/2;
            WheelCenter(1,2) = obj.yPos + sind(obj.heading)*obj.length/3 + sind(obj.heading+90)*obj.width/2;
            %FrontRight
            WheelCenter(2,1) = obj.xPos + cosd(obj.heading)*obj.length/3 - cosd(obj.heading+90)*obj.width/2;
            WheelCenter(2,2) = obj.yPos + sind(obj.heading)*obj.length/3 - sind(obj.heading+90)*obj.width/2;
            %BackLeft
            WheelCenter(3,1) = obj.xPos - cosd(obj.heading)*obj.length/3 + cosd(obj.heading+90)*obj.width/2;
            WheelCenter(3,2) = obj.yPos - sind(obj.heading)*obj.length/3 + sind(obj.heading+90)*obj.width/2;
            %BackRight
            WheelCenter(4,1) = obj.xPos - cosd(obj.heading)*obj.length/3 - cosd(obj.heading+90)*obj.width/2;
            WheelCenter(4,2) = obj.yPos - sind(obj.heading)*obj.length/3 - sind(obj.heading+90)*obj.width/2;
            
            %Calc max turn rate for Doppler
            turnRate = obj.vel/(2*pi*obj.rTire); % turns per second
            
            %v_wheeel in range vtarget + [-vd, +vd]
            %Smallest velocity in middle, increasing to the sides
            %Smallest RCS in middle (low v), larger on the sides (high v)
            %Area of a Ring with const. v on the wheel: A=pi*(r1^2 - r2^2)
            % A ~ dA/dr ~ 2dr with dr = r2-r1
            % => RCS ~ 2r   and   vd ~ r
            
            %0. Generate 2D map
            
            %1. Impose probability distributions across r/phi dimensions
            
            %2. Sample random Positions around Wheel center
            
            %3. Calculate RCS/vel from Position
            
            %4. RadarTargets at sampled Positions
                        
            
            
            %--------------------------------------------------------------
            % RCS
            % Calculate RCS dependent on hitting angle
            hittingAngle = abs(obj.normAngle(Contour(:,3)-DOA));
            RCSangle = rand(size(hittingAngle)).* hittingAngle/ReceptionAngle;
            scatter(Contour(:,1),Contour(:,2),[], 'r')
            Rayleigh = raylrnd(1:1000);
            Pdistribution = Rayleigh/max(Rayleigh);
            sigma = obj.RCS * RCSangle' .* (1- Pdistribution(ceil(rand(size(hittingAngle))*1000))); % TODO: obj.RCS(round(heading)) !!!!!!
            
            % Collect all Car Scattering Points
            obj.CarTarget = phased.RadarTarget('Model','Swerling2','MeanRCS',sigma,...
                    'PropagationSpeed',fmcw.c0,'OperatingFrequency',fmcw.f0);
            
            Elref = ones(size(hittingAngle))*fmcw.height; %TODO Add height !!!!!!!!!!
            obj.TargetPlatform = phased.Platform('InitialPosition',[Contour(:,1)'; Contour(:,2)'; Elref'], ...
                    'OrientationAxesOutputPort',true, 'Velocity', [cos(obj.heading)*obj.vel*ones(size(Contour(:,1)'));sin(obj.heading)*obj.vel*ones(size(Contour(:,1)'));zeros(size(Contour(:,1)'))], 'Acceleration', [0;0;0]);

        end
        
        
        
        %% Move Target Platform
        function [post,velt,axt] = move(obj, tsamp, ~)
            [post,velt,axt] = obj.TargetPlatform(tsamp);
        end
        
        
        %% Reflect incoming Signal
        function RXsig = reflect(obj, xtrans, angle)
            RXsig = obj.CarTarget(xtrans, true);
        end
        
    end
end

