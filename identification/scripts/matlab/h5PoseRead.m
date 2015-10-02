function [ table, board ] = h5PoseRead( h5File )
%h5PoseRead Summary of this function goes here
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
    table = hdf5read(h5File, '/H_table_from_reference_camera');
    table = table.';
    board = hdf5read(h5File, '/board_frame_offset');
    board = board.';
    if sum(table(:)) == 0 || sum(board(:)) == 0
        error('Empty file.')
    end
end

