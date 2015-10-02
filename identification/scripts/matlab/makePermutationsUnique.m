function vec = makePermutationsUnique(vec)
    for iii = 1:size(vec,1)-1
        
        jjj = iii+1;
        while (jjj <= size(vec,1))
            is_same = 1;
            for kkk = 1:size(vec,2)
               if (vec(iii,kkk) ~= vec(jjj,kkk))
                   is_same = 0;
               end
            end
            
            if (is_same == 1)
                vec(jjj,:) = [];
            else
                jjj = jjj + 1;
            end
        end
    end

end