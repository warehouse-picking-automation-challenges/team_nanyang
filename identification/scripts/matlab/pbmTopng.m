function [ im1 ] = pbmTopng( pbmAddr, saveAddr, bitDepth )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

    if (nargin == 0)
		error('3 inputs required: pbm image to convert, number of bits(optional),  destination folder(optional)')
    end
    
	[folder, name, ext] = fileparts(pbmAddr);
    if (nargin < 3)
        bitDepth = 8;
        if (nargin ==1)
            saveAddr = strcat(folder, '\', name,  '8bit.png');
        end
    end
    
    im1 = imread(pbmAddr);
    if bitDepth == 8
        imwrite(255-255*im1, saveAddr, 'Bitdepth', 8);
    elseif bitDepth == 1
            imwrite(1-im1, saveAddr, 'Bitdepth', 1);
    elseif isnumeric(bitDepth)
            error('bitDepth can only be 8(default) or 1!')
    else
        error('3rd arg must be bitDepth.')
    end
    
end

