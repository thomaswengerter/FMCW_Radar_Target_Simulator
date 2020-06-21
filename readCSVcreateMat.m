function createMat(folder)
% FUNCTION Read all csv files and combine them to Matlab struct
%   For each folder contained in Input "folder", all containing csv files
%   are collected in a .mat file containing a struct "Features". The individual
%   targets can be called with Features.Bike2 or Features.Pedestrian3.

% Check for all target types in "folder"
targets = dir(folder); 
targets = targets(~startsWith({targets.name}, '.'));

Features = struct;
% For each target type...
for i = 1:length(targets)
    path = [folder,targets(i).name,'/'];
    % Read available measurement files
    measfiles = dir([path,'/',targets(i).name(1),'label*']);

    
    % Fill Cell with all avail measurements
    Cell = {};
    fields = [];
    for file = 1:length(measfiles)
        RD = readmatrix([path,'/',targets(i).name,num2str(file)]);
        label = readmatrix([path,'/',targets(i).name(1),'label',num2str(file)]);
        fields = [fields; [targets(i).name,num2str(file)]];
        Cell{file,1} = RD;
        Cell{file,2} = label;
    end
    
    
    %Convert Cell to Features Struct and SAVE
    Features = cell2struct(Cell,fields,1);
    save([path,'Features',targets(i).name], 'Features')    
    
end

end

