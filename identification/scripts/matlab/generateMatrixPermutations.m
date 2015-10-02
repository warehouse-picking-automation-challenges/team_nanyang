function P = generateMatrixPermutations(n)
% http://math.stackexchange.com/questions/7840/finding-all-n%C3%97n-permutation-matrices
    seq = linspace(1,n,n);
    vec = [];
    [seq vec] = generatePermutations(n, seq, vec);
    vec = makePermutationsUnique(vec);
    
    for iii = 1:size(vec,1)
        P{iii} = zeros(n,n);
        for jjj = 1:size(vec,2)
           P{iii}(jjj,vec(iii,jjj)) = 1; 
        end
    end
end