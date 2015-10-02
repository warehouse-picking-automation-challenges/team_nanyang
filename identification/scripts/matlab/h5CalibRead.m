function [ camToRef, camIntrins, camDist ] = h5CalibRead( h5File, camToFind )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

    if (nargin == 0)
		error('Where is the h5 file?')
    end

    value = H5F.is_hdf5(h5File);
    if value == 0
        error('File given is not a h5 file!')
    end

    %{
    fileinfo = hdf5info(h5File);
    toplevel = fileinfo.GroupHierarchy;
    dataset1 = toplevel.Datasets(1);
    %}
    
    trans = strcat('/H_', camToFind, '_from_NP5');
    intrins = strcat('/', camToFind, '_rgb_K');
    dis = strcat('/', camToFind, '_rgb_d');
    
    camToRef = hdf5read(h5File, trans);
    camToRef = camToRef.';
    camIntrins = hdf5read(h5File, intrins);
    camIntrins = camIntrins.';
    camDist = hdf5read(h5File, dis);
    camDist = camDist.';
    
end

