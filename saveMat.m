function saveMat(RD, label, targetType, targetNo, SimDataPath)
%FUNCTION Save RDmap and label to .mat file
%   targetType: 'Pedestrian', 'Bicycle', 'Car', 'Syntetic'
%   targetNo:   target file index (for Bicycle3.mat)
%   SimDataPath: Path to sim result folder

folder = [SimDataPath, targetType, '/'];
save([folder, targetType, num2str(targetNo)],'RD');
save([folder, targetType(1), 'label', num2str(targetNo)],'label');

end

