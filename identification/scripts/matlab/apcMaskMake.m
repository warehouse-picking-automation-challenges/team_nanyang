function [ items, itemsCount ] = apcMaskMake( objDir, outDir )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if (nargin < 2)
            error('Please specify Directories!')
    end
    
    %check if addressout contains '\' at end.
    [a, b, c] = fileparts(objDir);
    if ~isempty(b)
        objDir = strcat(objDir, '\');
    end
    %check if addressout contains '\' at end.
    [a, b, c] = fileparts(outDir);
    if ~isempty(b)
        outDir = strcat(outDir, '\');
    end

    maskFolder = strcat(objDir, 'mask\');   %folder containing the mask images
    rgbFolder = strcat(objDir, 'rgb\');     %folder containing the rgb images
    %labelsFolder = strcat(objDir, 'labels\')
    maskDir = dir(maskFolder);
    maskCount = (size(maskDir,1)-2);    %no. of mask images
    maskNames = cell((maskCount-2),1);  %cell array to store mask names
    labelNames = cell((maskCount-2),1); %cell array to store label names
    items = cell((maskCount-2),1);      %cell array to store the item names in each image
    itemsCount = cell(1,1);             %cell array to store the number of items in each image
    
    % Create output directories
    mkdir(strcat(outDir, 'mask'));
    mkdir(strcat(outDir, 'rgb'));
    mkdir(strcat(outDir, 'labels'));
    
    for i = 1:maskCount
        maskNames{i} = maskDir(i+2).name;           %get mask names
        str1 = strsplit(maskDir(i+2).name, '.');
        labelNames(i) = strcat(str1(1), '.txt');    %make label names from mask names
    end
    
    for i=1:size(labelNames,1)
        str1 = strcat(objDir, 'labels\', labelNames{i});
        fid = fopen(str1); %(strcat(labelsFolder, labelNames(i)));  %read label txt file, extract all items and itemCount
        objCount = 0;
        while (1)
            tline = fgets(fid);
            if tline~=-1
                objCount = objCount+1;
            else
                break;
            end
        end
        itemsCount{i,1} = objCount;
        fclose(fid);
        %str1 = strcat(objDir, 'labels\', labelNames{i});
        fid = fopen(str1);
        for j = 1:objCount
            tline = fgets(fid);
            items{i,j} = tline;
        end
        fclose(fid);
    end
    
    for i=1:maskCount
        currMask = imread(strcat(maskFolder, maskNames{i}));      %read in mask image
        maskRGB = imread(strcat(rgbFolder, maskNames{i}));        %read in corresponding rgb image
        
        %Check if rgb, gray or invalid.
        if size(currMask,3)==3
            currMask = rgb2gray(currMask);
        else if size(currMask,3)~=1
                error('mask folder contains neither rgb or grayscale image')
            end
        end
        
        [row, col, val] = find(currMask);     %find the non-zero pixel values
        pixVals = unique(val);              %get the unique values (1st occurence of values)
        for j=1:itemsCount{i,1}
            combo = combnk(1:itemsCount{i,1},3);        %for no. 1 through itemCount, obtain all possible combinations of 3 items
            for k=1:(size(combo,1))                     %iterate through all combinations
                newMask = zeros(480, 640, 'uint8');    %empty image
                for l=1:(size(val,1))
                    if val(l)==pixVals(combo(k,1)) || val(l)==pixVals(combo(k,2)) || val(l)==pixVals(combo(k,3))    %obtain x,y positions of pixels corresponding to values of current combination
                        newMask(row(l), col(l)) = val(l);
                    end
                end
                str1 = strsplit(maskNames{i}, '.');
                str2 = str1{1};%str2 = cellstr(str1(1));
                %str2 = strcat(outDir, 'mask\image_001_001.png');
                %str3 = strcat(outDir, 'mask\', str2, '_00', num2str(k), '.png');
                if k<10
                    imwrite(newMask, strcat(outDir, 'mask\', str2, '_00', num2str(k), '.png'));%, 'Bitdepth', 8);              %save new mask image of 3 items combo
                    imwrite(maskRGB, strcat(outDir, 'rgb\', str2, '_00', num2str(k), '.png'));
                    fid = fopen(strcat(outDir, 'labels\', str2, '_00', num2str(k), '.txt'), 'wb');
                else if k<100
                        imwrite(newMask, strcat(outDir, 'mask\', str2, '_0', num2str(k), '.png'));
                        imwrite(maskRGB, strcat(outDir, 'rgb\', str2, '_0', num2str(k), '.png'));
                        fid = fopen(strcat(outDir, 'labels\', str2, '_0', num2str(k), '.txt'), 'wb');
                    else
                        imwrite(newMask, strcat(outDir, 'mask\', str2, '_', num2str(k), '.png'));
                        imwrite(maskRGB, strcat(outDir, 'rgb\', str2, '_', num2str(k), '.png'));
                        fid = fopen(strcat(outDir, 'labels\', str2, '_', num2str(k), '.txt'), 'wb');
                    end
                end
                fprintf(fid, '%s\r%s\r%s\r', items{i,combo(k,1)}, items{i,combo(k,2)}, items{i,combo(k,3)}); %write the 3 items into text
                fclose(fid);
            end
        end
    end
    
end

