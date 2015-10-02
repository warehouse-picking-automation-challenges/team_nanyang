function [ depthMat ] = h5DepthRead( h5File )
%h5DepthRead Summary of this function goes here
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
    depthMat = hdf5read(h5File, '/depth');
    depthMat = depthMat.';
    if sum(depthMat(:)) == 0
        error('Empty file.')
    end
end

