libraryAddressFile = '../../params/object_library_address.txt';

fid = fopen(libraryAddressFile);
libraryAddress = fgetl(fid);
fclose(fid);

itemList = dir(libraryAddress);

for iii = 3:size(itemList, 1)
    
    % Define and/or create subdirectory for all RGB images
    sub_dir = [libraryAddress '\' itemList(iii).name '\rgbd' '\rgb'];
    mkdir(sub_dir);
    
    disp(['moving for item <' itemList(iii).name '>...']);
    
    % Move any PBM masks from main directory to subdirectory
    contentsList = dir([libraryAddress '\' itemList(iii).name '\rgbd']);
    for jjj = 1:size(contentsList,1)
       if (size(contentsList(jjj).name, 2) > 4 & ~contentsList(jjj).isdir)
           ext = contentsList(jjj).name(size(contentsList(jjj).name,2)-2:size(contentsList(jjj).name,2));
           if strcmp(ext, 'jpg')
               src = [libraryAddress '\' itemList(iii).name '\rgbd' '\' contentsList(jjj).name];
               dst = [sub_dir '\' contentsList(jjj).name];
               movefile(src,dst);
           end
       end
    end
end
