% This script is used to explore how empirically determined 2-way identification probabilities can
% be used to predict probabilities with multiple blobs and labels.

blob_count = 3
class_count = blob_count

if (blob_count == 2)

    % Marginal probabilities can be determined from confidence mappings
    Prob = zeros(blob_count,class_count);
    Prob(1,1) = 0.614; % Probability of blob 1 belonging to class 1
    Prob(2,1) = 0.538; % Probability of blob 2 belonging to class 1
    
    for iii = 1:blob_count
        for jjj = 1:class_count
            if (Prob(iii,jjj) == 0)
                Prob(iii,jjj) = 1 - Prob(iii,1);
            end
        end
    end
    
    mat = zeros(blob_count, class_count, blob_count);
    
    for iii = 1:blob_count
        for jjj = 1:blob_count
            for kkk = 1:blob_count
                if (iii == jjj)
                    mat(iii,jjj,kkk) = 0;
                else
                    % mat(iii,jjj,kkk) = 3;
                end
                
            end
        end
    end
    
    % This matrix stores the pairwise prob for blob 1
    mat(:,:,1) = [1 Prob(1,1); Prob(1,2) 1];
    % And this for blob 2
    mat(:,:,2) = [1 Prob(2,1); Prob(2,2) 1];

else
    % 3/3 case:
    % Blobs 1-3
    % Classes A-C

    % Marginal probabilities can be determined from confidence mappings
    P_1AB = 0.6135; % Probability of blob 1 belonging to class A rather than class B
    P_1BC = 0.4624; % Probability of blob 1 belonging to class B rather than class C
    P_1AC = 0.5375; % Probability of blob 1 belonging to class A rather than class C
    
    P_2AB = 0.5378; % Probability of blob 2 belonging to class A rather than class B
    P_2BC = 0.3852; % Probability of blob 2 belonging to class B rather than class C
    P_2AC = 0.4622; % Probability of blob 2 belonging to class A rather than class C
    
    P_3AB = 0.5392; % Probability of blob 3 belonging to class A rather than class B
    P_3BC = 0.4606; % Probability of blob 3 belonging to class B rather than class C
    P_3AC = 0.4999; % Probability of blob 3 belonging to class A rather than class C

    % These matrices stores the pairwise probs for blobs 1-3
    mat = zeros(blob_count, blob_count, blob_count);
    
    mat(:,:,1) = [1 P_1AB P_1AC; (1-P_1AB) 1 P_1BC; (1-P_1AC) (1-P_1BC) 1];
    mat(:,:,2) = [1 P_2AB P_2AC; (1-P_2AB) 1 P_2BC; (1-P_2AC) (1-P_2BC) 1];
    mat(:,:,3) = [1 P_3AB P_3AC; (1-P_3AB) 1 P_3BC; (1-P_3AC) (1-P_3BC) 1];
    
end

mat

% Then create the LPP
lpp = zeros(blob_count,class_count);
for iii = 1:blob_count
    denom = 0;
    for jjj = 1:class_count
        vec = mat(jjj,:,iii);
        denom = denom + prod(vec);
    end
    for jjj = 1:class_count
        vec = mat(jjj,:,iii);
        lpp(iii,jjj) = prod(vec)/denom;
    end
end

lpp

% Generate configurations
P = generateMatrixPermutations(blob_count); % , class_count

config_probs = [];

for c = 1:size(P,2)
    local_mat = lpp * P{c};
    prob = 1.0;
    for iii = 1:size(local_mat,1)
        prob = prob * local_mat(iii,iii);
    end
    config_probs = [config_probs prob];
end
config_probs = config_probs / sum(config_probs)
[v i] = max(config_probs);
best_match = P{i}

% Figure out most probable labels for each blob
id_probs = zeros(blob_count,class_count);
for iii = 1:blob_count
    for c = 1:size(P,2)
        for jjj = 1:class_count
            if (P{c}(iii,jjj) > 0)
                id_probs(iii,jjj) = id_probs(iii,jjj) + config_probs(c);
            end
        end
    end
end

id_probs
