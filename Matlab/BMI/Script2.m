%Just run this script

close all
clear all
clc

load('monkeydata0.mat');

NumDir=8;
NeurUnits=98;
%vectg=[18 27 31 44 69 81 84 90 93 97];
training_data=trial(1:50,:); %%% Trainind data
test_data=trial(51:70,:);    %%% Test data




avPosition=[0 0]';
for tr=1:size(test_data,1)
    for dir=1:8
        interval = 301:size(test_data(tr,dir).spikes,2)-100;
        avPosition = avPosition + mean(test_data(tr,dir).handPos(1:2,interval),2);
    end
end
avPosition = avPosition / size(test_data,1) / 8;

meanSqError = 0;
meanSqError_trivial = 0;

figure
hold on
axis square
grid

decoderParameters = positionEstimatorTraining(training_data);
for tr=1:size(test_data,1)
    display(['Decoding block ',num2str(tr),' out of ',num2str(size(test_data,1))]);
    pause(0.001)
    for dir=randperm(8)
        decodedHandPos = [];
  
        times=320:20:size(test_data(tr,dir).spikes,2)-100;
        for t=times
            past_current_trial.trialId = test_data(tr,dir).trialId;
            past_current_trial.spikes = test_data(tr,dir).spikes(:,1:t);%% it will be modified in classifier_k in 201:t
            past_current_trial.decodedHandPos = decodedHandPos;
            past_current_trial.startHandPos = test_data(tr,dir).handPos(1:2,201);

            [decodedPos newDecoderParameters] = position_estimator(decoderParameters, past_current_trial);
            decoderParameters = newDecoderParameters;
            
            %Computation of the mean square error             
            meanSqError_trivial = meanSqError_trivial + norm(test_data(tr,dir).handPos(1:2,t) - avPosition(1:2))^2;
            meanSqError = meanSqError + norm(test_data(tr,dir).handPos(1:2,t) - decodedPos(1:2)')^2;
            
            decodedHandPos = [decodedHandPos decodedPos'];
            
            plot(decodedPos(1), decodedPos(2), 'r.')
            plot(test_data(tr,dir).handPos(1,t),test_data(tr,dir).handPos(2,t),'b.');
        end
        plot(decodedHandPos(1,:),decodedHandPos(2,:),'r');
        plot(test_data(tr,dir).handPos(1,times),test_data(tr,dir).handPos(2,times),'b')
    end
end
accuracy = 1 - meanSqError / meanSqError_trivial;
RMSE = sqrt(meanSqError/50/8) %% modify in function of the number of data used

