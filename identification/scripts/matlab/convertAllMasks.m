displayResults = 1;

libraryAddressFile = '../../params/object_library_address.txt';

fid = fopen(libraryAddressFile);
libraryAddress = fgetl(fid);
fclose(fid);

itemList = dir(libraryAddress);

for iii = 3:size(itemList, 1)
    
    % Define and/or create subdirectory for all PBM masks
    sub_dir = [libraryAddress '\' itemList(iii).name '\rgbd' '\masks'];
    mkdir(sub_dir);
    
    % Define and/or create subdirectory for all PNG masks
    maskList = dir(sub_dir);
    mkdir([libraryAddress '\' itemList(iii).name '\rgbd' '\mask']);
    
    disp(['processing for item <' itemList(iii).name '>...']);
    
    % Loop through all PBM masks and move them into the PNG directory
    for jjj = 3:size(maskList,1)
       maskInputName = [sub_dir '\' maskList(jjj).name];
       maskOutputName = maskInputName;
       maskOutputName = strrep(maskOutputName, 'masks', 'mask');
       maskOutputName = strrep(maskOutputName, '.pbm', '.png');
       
       disp(['processing item ' num2str(iii-2) '/' num2str(size(itemList, 1)-2) '; image ' num2str(jjj-2) '/' num2str(size(maskList,1)-2)]);
       result = pbmTopng( maskInputName, maskOutputName, 8 );
       if displayResults
           imshow(result);
       end
    end
    
end
