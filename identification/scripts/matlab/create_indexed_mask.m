input_name = 'C:\Users\Stephen\Documents\Data\APC\field_data\subset\elmers_washable_no_run_school_glue\rgbd\mask\NP1_27_mask_col.png';
output_name = 'C:\Users\Stephen\Documents\Data\APC\field_data\subset\elmers_washable_no_run_school_glue\rgbd\mask\NP1_27_mask.png';

image = imread(input_name);

mask = zeros(size(image,1), size(image,2), 'uint8');

shift = 80;

for iii = 1:size(image,1)
    for jjj = 1:size(image,2)
        if (image(iii,jjj,1) > 0)
            mask(iii,jjj) = mask(iii,jjj) + shift;
        end
        if (image(iii,jjj,2) > 0)
            mask(iii,jjj) = mask(iii,jjj) + shift;
        end
        if (image(iii,jjj,3) > 0)
            mask(iii,jjj) = mask(iii,jjj) + shift;
        end
    end
end

imshow(mask);

% mask = mask / shift;

imwrite(mask, output_name);