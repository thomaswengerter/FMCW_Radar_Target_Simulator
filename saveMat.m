function saveMat(RD, label, targetType, targetNo, SimDataPath)
%FUNCTION Save RDmap and label to .mat file
%% called from: TargetSimulation.m
%   targetType: 'Pedestrian', 'Bicycle', 'Car', 'Szenario3'
%   targetNo:   target file index (for Szenario3_1.mat, Bicycle_3.mat)
%   SimDataPath: Path to sim result folder

savePics = true;

folder = [SimDataPath, targetType, '/'];

save([folder, targetType,'_', num2str(targetNo)],'RD');
save([folder, targetType, '_Label_', num2str(targetNo)],'label');


%% YOLO pics
if savePics
    rangeDoppler = RD;

    % Get rid of first range bins
    rangeDoppler = rangeDoppler(6:end, :,:);

    % Convert RD Spectrum to grey scale image, max. value -20, min. value
    % -85. Output values are within 0 and 1. Max. and min. values are
    % adjusted according to the data, so that they hava a dynamic range of
    % around 65 dB
    rD_gs = mat2gray(rangeDoppler, [-185, -120]);  %[-185, -120], [-175, -110]

    % Max. across angle bins
    rD_gs = max(rD_gs,[], 3);

    % Scale them between 0 and 255 and flip
    rD_gs = uint8(flipud(rD_gs*255));

    [~,~] = mkdir('YOLOpics');
    imwrite(rD_gs, ['YOLOpics/', targetType, '_', num2str(targetNo),'.png'])
end
end

