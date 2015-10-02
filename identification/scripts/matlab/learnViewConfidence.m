% This script reads in data output from the identification_library_test
% program, in order to learn how to appropriately map NNDRs into
% probabilisitic (confidence) estimates for VIEW MATCHING.

if (is_octave > 0)
	close all; clear all; clc;
	is_octave = 1;
else
	close all; clear all; clc;
	is_octave = 0;
end

histogramDivisions = 10;
min_samples = 50;
distanceMetricCount = 4;

for distanceMetric = 1:distanceMetricCount
    
    resultsAddress = ['../../output/view_ratio_data_' num2str(distanceMetric-1) '.txt'];
    successfulRatios = sort(load(resultsAddress));
    successfulRatios = successfulRatios(successfulRatios ~= 1.0); % Actually want to ignore ratios of 1.0 exactly as they are totally ambiguous
    
    histogramData = zeros(histogramDivisions,2);
    
    % Tally ratios
    for iii = 1:size(successfulRatios,1)
        if (successfulRatios(iii,1) < 1.0) % If ratio was, as expected, less than 1
            val = successfulRatios(iii,1);
            idx = ceil(val*histogramDivisions);
            idx = max(idx, 1);
            histogramData(idx, 1) = histogramData(idx, 1) + 1;
        elseif (successfulRatios(iii,1) > 1.0)
            val = 1/successfulRatios(iii,1);
            idx = ceil(val*histogramDivisions);
            idx = max(idx, 1);
            histogramData(idx, 2) = histogramData(idx, 2) + 1;
        end 
    end
    
    % Remove small samples
    for iii = 1:size(histogramData,1)
        if (histogramData(iii,1) + histogramData(iii,2) < min_samples)
            histogramData(iii,1) = 0;
            histogramData(iii,2) = 0;
        end
    end
    
    histogramSumm = histogramData(:,1) ./ (histogramData(:,1)+histogramData(:,2));
    
    % ============
    % Establish confidence points
    pointData = histogramSumm;
    pointData(~isnan(pointData)) = min(pointData(~isnan(pointData)), 1.0);
    pointData(~isnan(pointData)) = max(pointData(~isnan(pointData)), 0.0);
    
    first_valid = 1
    while (isnan(pointData(first_valid)))
       first_valid = first_valid + 1;
       if (first_valid > size(pointData,1)) 
           break;
       end
    end
    
    last_valid = size(pointData,1);
    while (isnan(pointData(last_valid)))
       last_valid = last_valid - 1;
       if (last_valid < 1) 
           break;
       end
    end
    
    min_val = 1.0;
    max_val = 0.0;
    for iii = first_valid:last_valid
        if (~isnan(pointData(iii)))
            min_val = max(0.0,min(pointData(iii), min_val));
            max_val = min(1.0,max(pointData(iii), max_val));
        end
        pointData(iii) = min_val;
    end
    pointData(1:first_valid-1) = max_val;
    %pointData(last_valid+1:end) = min(0.0,min_val);
    y = zeros(1,histogramDivisions+1);
    y(1) = pointData(1) + (pointData(1)-pointData(2))/2;
    for iii = 2:histogramDivisions
       y(iii) =  (pointData(iii-1) + pointData(iii))/2;
    end
    y(histogramDivisions+1) = pointData(histogramDivisions) + (pointData(histogramDivisions)-pointData(histogramDivisions-1))/2.0;
    % ============
    
    % Apply confidence to test
    % Column 1: ratio
    % Column 2: correctness
    % Column 3: estimated probability / confidence
    evaluationRatios = [min(successfulRatios, 1./successfulRatios) (successfulRatios<1.0)];
    evaluationRatios = sortrows(evaluationRatios, 1);
    
    evaluationRatios = [evaluationRatios zeros(size(evaluationRatios,1), 1)];
    for iii = 1:size(evaluationRatios,1)
        idx = floor(evaluationRatios(iii,1)*histogramDivisions)+1;
        idx = min(idx, histogramDivisions);
        scale = evaluationRatios(iii,1) - (idx-1)/histogramDivisions;
        evaluationRatios(iii,3) = (1-scale)*y(idx) + scale*y(idx+1);
    end
    
    histogramData_2 = zeros(histogramDivisions,2);
    for iii = 1:size(evaluationRatios,1)
        idx = ceil(evaluationRatios(iii,3)*histogramDivisions);
        idx = max(idx, 1);
        idx = min(idx, histogramDivisions);
        
        if (evaluationRatios(iii,2) == 1.0)
            histogramData_2(idx,1) = histogramData_2(idx,1) + 1;
        else
            histogramData_2(idx,2) = histogramData_2(idx,2) + 1;
        end
    end
    histogramSumm_2 = histogramData_2(:,1) ./ (histogramData_2(:,1)+histogramData_2(:,2))
    
    modelEffectiveness = 0;
    binCount = 0;
    for iii = 1:size(histogramSumm_2,1)
        
        expectedRate = (iii-0.5)/size(histogramSumm_2,1);
        measuredRate = histogramSumm_2(iii,1);
        
        if ~isnan(histogramSumm_2(iii,1))
            modelEffectiveness = modelEffectiveness + abs(measuredRate-expectedRate);
            binCount = binCount + 1;
        end
    end
    
    edit  = modelEffectiveness / binCount;
    modelEffectiveness = 1 - modelEffectiveness;
    modelScores(distanceMetric) = modelEffectiveness;
    
    figure(distanceMetric)
    ratioScale = linspace(0.00,1.00, histogramDivisions+1)';
    bar_x = linspace(0+1/(2*histogramDivisions), 1-1/(2*histogramDivisions), histogramDivisions);
    N = histc(histogramSumm, ratioScale);
    bar(bar_x, [histogramSumm histogramSumm_2], 'grouped');
    hold on;
    
    plot( ratioScale, y, 'LineWidth', 3, 'Color', [0.5 0.5 1.0]);
    plot( ratioScale, ratioScale, 'LineWidth', 3, 'Color', [1.0 0.5 0.5]);
    axis([0 1 0 1.2]);
    xlabel('Score');
    ylabel('Success Rate');
    legend('Identity Match Ratio', 'Confidence Level');
    
    outputAddress = ['../../output/view_ratio_pts_' num2str(distanceMetric-1) '.txt'];
    fod = fopen(outputAddress, 'w');
    fprintf(fod, '%f ', y);
    fprintf(fod, '\n');
    fclose(fod);
end

pause(3.0)
modelScores
