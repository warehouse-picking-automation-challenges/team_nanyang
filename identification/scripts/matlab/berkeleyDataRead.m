function [ depthMat, table, camToRef, camIntrins, camDist ] = berkeleyDataRead( rgbIm )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

    if (nargin == 0)
		error('Where is the image?')
    end

    [folder, name, c] = fileparts(rgbIm);
    camName = sscanf(name, '%c', 3);
    angle = sscanf(name, '%*c%*c%*c%*c%s');
    depthh5 = strcat(folder, '\', name, '.h5');
    if exist(depthh5, 'file') == 2
        depthMat = h5DepthRead(depthh5);
    else
        depthh5 = strcat(folder, '\depth\', name, '.h5');
        if exist(depthh5, 'file') == 2
            depthMat = h5DepthRead(depthh5);
        else
            error('Depth image h5 file not found!')
        end
    end
    poseh5 = strcat(folder, '\poses\NP5_', angle, '_pose.h5');
    table = h5PoseRead(poseh5);
    [camToRef, camIntrins, camDist] = h5CalibRead( strcat(folder,'\calibration.h5'), camName);
    
end

