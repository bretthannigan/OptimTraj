function param = readStoFile(varargin)

    p = inputParser;
    addOptional(p, 'filePath', '', @ischar);
    parse(p, varargin{:});
    filePath = p.Results.filePath;

    if isempty(filePath)
        [fileName, pathName] = uigetfile({'*.sto', 'OpenSim Storage Files (*.sto)'}, ...
                                 'Select the OpenSim Storage File', 'MultiSelect', 'off');
        filePath = fullfile(pathName, fileName);
    else
        [~, fileName, fileExt] = fileparts(filePath);
        fileName = strcat(fileName, fileExt);
    end
    stoData = importdata(filePath);
    param.stoFileName = fileName;
    param.stateNames = stoData.colheaders(2:end)';
    param.time = stoData.data(:,1)';
    param.states = stoData.data(:,2:end)';

end