function saveMat(RD, label, targetType, targetNo, SimDataPath)
%FUNCTION Save RDmap and label to .mat file
%   targetType: 'Pedestrian', 'Bicycle', 'Car', 'Syntetic'
%   targetNo:   target file index (for Bicycle3.mat)
%   SimDataPath: Path to sim result folder

folder = [SimDataPath, targetType, '/'];
save([folder, targetType, num2str(targetNo)],'RD');
save([folder, targetType(1), 'label', num2str(targetNo)],'label');


%% YOLO pics
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

% Scale them between 0 and 255
rD_gs = uint8(rD_gs*255);

[~,~] = mkdir('SimulationData/YOLOpics');
imwrite(rD_gs, ['SimulationData/', targetType, num2str(targetNo),'.png'])

end

