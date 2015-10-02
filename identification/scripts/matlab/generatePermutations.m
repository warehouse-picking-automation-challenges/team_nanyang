function [A vec] = generatePermutations(n, A, vec)
% http://en.wikipedia.org/wiki/Heap%27s_algorithm
if n ~= 1
    for i = 1:n
        [A vec] = generatePermutations(n-1, A, vec);
        if mod(n,2) == 1
            j = 1;
        else
            j = i;
        end
        tmp = A(1,j);
        A(1,j) = A(1,n);
        A(1,n) = tmp;
    end
else
    vec = [vec; A];
end

end