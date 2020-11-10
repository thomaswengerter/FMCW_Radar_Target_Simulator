function Rmap = generateObstructionMap(Targets, fmcw)
%   Generate Filter for obstructed Scatterers
%   
%   OUTPUT:
%   Rmap = [tstep, azimuth (-90,+90), Range, (max height obstructed, target ID)]

%Rmap = zeros(fmcw.L, 181, length(fmcw.rangeBins), 2);
Rmap = zeros(181, length(fmcw.rangeBins), 2);


sz = size(Targets);
for i = 1:sz(1)
    target = Targets{i,2};
    if strcmp(Targets{i,1}(1:end-1),'Pedestrian') 
        % INDEX 1
        %for chirp = 1:fmcw.L
        target.release();
        [post,velt,axt] = move(target,fmcw.chirpInterval,target.InitialHeading);
        azi = atand(post(2,:)./post(1,:));
        aziCover = round(min(azi)): round(max(azi)); %target azi width
        RidxCover = ceil(min(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR); %target min range
        if RidxCover<=0 && floor(max(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR)<=0
            RidxCover = length(fmcw.rangeBins)+1; %Target behind radar, no obstruction
        elseif RidxCover<=0 && floor(max(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR)>0
            RidxCover = 1; %Target right in front of radar, obstruction
        end
        heightCover = target.Height; %target height
        for a = aziCover
            for idx = RidxCover:length(fmcw.rangeBins)
                if (a+90)<=size(Rmap,2)&& (a+90)>0
                    if (Rmap(a+90,  idx, 2) == 0) || (Rmap(a+90, idx, 1) <= heightCover)
                        % No larger Element in front
                        Rmap(a+90,  idx, :) = [heightCover, 1]; %Position obstruction in map                  
                    end
                end
            end
        end
        
    elseif strcmp(Targets{i,1}(1:end-1),'Bicycle') 
        % INDEX 2
        %for chirp = 1:fmcw.L
        heightCover = target.Height; %target height
        for a = 1:length(target.aziCoverage)
            for idx = (ceil(target.rCoverage(a)/fmcw.dR)):length(fmcw.rangeBins)
                if (target.aziCoverage(a)+90)<= size(Rmap,2) && (target.aziCoverage(a)+90)>0
                    if (Rmap(target.aziCoverage(a)+90,  idx, 2) == 0) || (Rmap(target.aziCoverage(a)+90, idx, 1) <= heightCover)
                        % No larger Element in front
                        Rmap(target.aziCoverage(a)+90,  idx, :) = [heightCover, 2]; %Position obstruction in map                  
                    end
                end
            end
        end
       
        % MATLAB phased Toolbox
%         [post,velt,axt] = move(target,fmcw.chirpInterval,target.InitialHeading);
%         azi = atand(post(2,:)./post(1,:));
%         aziCover = round(min(azi)): round(max(azi)); %target azi width
%         RidxCover = ceil(min(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR); %target min range
%         if RidxCover<=0 && floor(max(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR)<=0
%             RidxCover = length(fmcw.rangeBins)+1; %Target behind radar, no obstruction
%         elseif RidxCover<=0 && floor(max(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR)>0
%             RidxCover = 1; %Target right in front of radar, obstruction
%         end
%         heightCover = max(post(3,:)); %target height
%         for a = aziCover
%             for idx = RidxCover:length(fmcw.rangeBins)
%                 if (a+90)<=size(Rmap,2)&& (a+90)>0
%                     if (Rmap(a+90,  idx, 2) == 0) || (Rmap(a+90, idx, 1) <= heightCover)
%                         % No larger Element in front
%                         Rmap(a+90,  idx, :) = [heightCover, 2]; %Position obstruction in map                  
%                     end
%                 end
%             end
%         end
        
    elseif strcmp(Targets{i,1}(1:end-1),'Vehicle')
        % INDEX 3
        %for chirp = 1:fmcw.L
        %[post,velt,axt] = move(target,tsamp,target.InitialHeading);
        %azi = atand(post(2,:)./post(1,:));
        %aziCover = round(min(azi)): round(max(azi)); %target azi width
        %RidxCover = floor(min(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR); %target min range
        heightCover = target.Height; %target height
        for a = 1:length(target.aziCoverage)
            for idx = (ceil(target.rCoverage(a)/fmcw.dR)):length(fmcw.rangeBins)
                if (target.aziCoverage(a)+90)<= size(Rmap,2) && (target.aziCoverage(a)+90)>0
                    if (Rmap(target.aziCoverage(a)+90,  idx, 2) == 0) || (Rmap(target.aziCoverage(a)+90, idx, 1) <= heightCover)
                        % No larger Element in front
                        Rmap(target.aziCoverage(a)+90,  idx, :) = [heightCover, 3+target.typeNr]; %Position obstruction in map                  
                    end
                end
            end
        end
    end
end
end

