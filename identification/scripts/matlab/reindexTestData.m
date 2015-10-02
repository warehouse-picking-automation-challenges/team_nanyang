input_dir = 'C:\Users\Stephen\Documents\GitHub\apc\identification\test_data\Test_data_16Mar2015';
new_start_idx = 51;

% [rgb mask depth label]
activate = [0 0 0 1]

if (activate(1) > 0)
    rgbDir = [input_dir '\' 'rgb'];
    rgbList = dir(rgbDir);

    idx = new_start_idx;
    for iii = 3:size(rgbList, 1)
        oldName = [rgbDir '\' rgbList(iii).name]
        newName = [rgbDir '\' sprintf('im_%06d.png', idx)]
        copyfile(oldName, newName);
        delete(oldName);
        idx = idx + 1;
    end
end

if (activate(2) > 0)
    maskDir = [input_dir '\' 'mask'];
    maskList = dir(maskDir);

    idx = new_start_idx;
    for iii = 3:size(maskList, 1)
        oldName = [maskDir '\' maskList(iii).name]
        newName = [maskDir '\' sprintf('im_%06d.png', idx)]
        copyfile(oldName, newName);
        delete(oldName);
        idx = idx + 1;
    end
end

if (activate(3) > 0)
    depthDir = [input_dir '\' 'depth'];
    depthList = dir(depthDir);

    idx = new_start_idx;
    for iii = 3:size(depthList, 1)
        oldName = [depthDir '\' depthList(iii).name]
        newName = [depthDir '\' sprintf('im_%06d.png', idx)]
        copyfile(oldName, newName);
        delete(oldName);
        idx = idx + 1;
    end
end

if (activate(4) > 0)
    labelDir = [input_dir '\' 'labels'];
    labelList = dir(labelDir);

    idx = new_start_idx;
    for iii = 3:size(labelList, 1)
        oldName = [labelDir '\' labelList(iii).name]
        newName = [labelDir '\' sprintf('im_%06d.txt', idx)]
        copyfile(oldName, newName);
        delete(oldName);
        idx = idx + 1;
    end
end