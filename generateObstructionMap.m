function Rmap = generateObstructionMap(Targets, fmcw)
%   Generate Filter for obstructed Scatterers
%   
%   OUTPUT:
%   Rmap = [tstep, azimuth (-90,+90), Range, (max height obstructed, target ID)]

Rmap = zeros(fmcw.L, 181, length(fmcw.rangeBins), 2);


sz = size(Targets);
for i = 1:sz(1)
    target = Targets{i,2};
    if strcmp(Targets{i,1}(1:end-1),'Pedestrian') 
        % INDEX 1
        for chirp = 1:fmcw.L
            target.release();
            [post,velt,axt] = move(target,fmcw.chirpInterval,target.InitialHeading);
            azi = atand(post(2,:)./post(1,:));
            aziCover = round(min(azi)): round(max(azi)); %target azi width
            RidxCover = floor(min(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR); %target min range
            heightCover = target.Height; %target height
            for a = aziCover
                for idx = RidxCover:length(fmcw.rangeBins)
                    if (Rmap(chirp, a+90,  idx, 2) == 0) || (Rmap(chirp, a+90, idx, 1) <= heightCover)
                        % No larger Element in front
                        Rmap(chirp, a+90,  idx, :) = [heightCover, 1]; %Position obstruction in map                  
                    end
                end
            end
        end
        
    elseif strcmp(Targets{i,1}(1:end-1),'Bicycle') 
        % INDEX 2
        for chirp = 1:fmcw.L
            [post,velt,axt] = move(target,fmcw.chirpInterval,target.InitialHeading);
            azi = atand(post(2,:)./post(1,:));
            aziCover = round(min(azi)): round(max(azi)); %target azi width
            RidxCover = floor(min(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR); %target min range
            heightCover = max(post(3,:)); %target height
            for a = aziCover
                for idx = RidxCover:length(fmcw.rangeBins)
                    if (Rmap(chirp, a+90,  idx, 2) == 0) || (Rmap(chirp, a+90, idx, 1) <= heightCover)
                        % No larger Element in front
                        Rmap(chirp, a+90,  idx, :) = [heightCover, 2]; %Position obstruction in map                  
                    end
                end
            end
        end
        
    elseif strcmp(Targets{i,1}(1:end-1),'Car')
        % INDEX 3
        for chirp = 1:fmcw.L
            %[post,velt,axt] = move(target,tsamp,target.InitialHeading);
            %azi = atand(post(2,:)./post(1,:));
            %aziCover = round(min(azi)): round(max(azi)); %target azi width
            %RidxCover = floor(min(sqrt(post(1,:).^2+post(2,:).^2))/fmcw.dR); %target min range
            heightCover = car.Height; %target height
            for a = 1:length(target.aziCoverage)
                for idx = target.rCoverage(a):length(fmcw.rangeBins)
                    if (Rmap(chirp, target.aziCoverage(a)+90,  idx, 2) == 0) || (Rmap(chirp, target.aziCoverage(a)+90, idx, 1) <= heightCover)
                        % No larger Element in front
                        Rmap(chirp, target.aziCoverage(a)+90,  idx, :) = [heightCover, 3+car.typeNr]; %Position obstruction in map                  
                    end
                end
            end
        end
    end
end
end

