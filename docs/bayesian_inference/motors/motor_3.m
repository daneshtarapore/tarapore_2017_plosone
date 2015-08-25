%Recursive bayesian estimation of the robots odometric output - distance
%traversed in last 10s
% using a Kalman filter approach


% Using a uni-variate gaussian distribution for state estimation and
% prediction

% Assume a Markovian state model of the hidden variable Theta(t) where t can indicate time or the observation index
% Theta(t+1) | Theta(t) follows N(Theta(t), Variance(t+1)); Prior Theta(0) = 0

% The Variance(t+1) could be ((t+1) - t) * ProcessNoiseVariance, so if you have not
% made an observation in sometime, the Variance in your prediction
% increases.


% Observation model
% X(t) | Theta(t) follows N(Theta(t), VarianceInMeasurement)  the
% VarianceInMeasurement is due to sensor noise and is analogous to the
% variance in likelihood


% Consider at time t we have the Kalman filter distribution of Theta(t) as
% N(mu(t), var(t))
% then the predictive distribution of Theta(t+1) given Theta(t) is
% Theta(t+1) | X(t) follows N(mu(t), var(t) + ProcessNoiseVariance)

% The ProcessNoiseVariance can be a constant or it can proportional to the
% time since last observation
% Example: ProcessNoiseVariance * (((t+1) - t)) 

% Correction

% mu(t+1) = (mu(t) / (var(t) + ProcessNoiseVariance) + x(t+1) / VarianceInMeasurement
%           __________________________________________________________________________    
%           1 / (var(t) + ProcessNoiseVariance)  + 1 / VarianceInMeasurement


% var(t+1) =                            1
%            __________________________________________________________________________
%            1 / (var(t) + ProcessNoiseVariance)  + 1 / VarianceInMeasurement  


% The correction of mu(t+1) can also be written as follows:

% mu(t+1) = mu(t) +  (var(t) + ProcessNoiseVariance)                      (X(t+1) - mu(t))
%                   ______________________________________________________   
%                   (var(t) + ProcessNoiseVariance + VarianceInMeasurement) 


% where         (var(t) + ProcessNoiseVariance) 
%            _______________________________________________________
%            (var(t) + ProcessNoiseVariance + VarianceInMeasurement)
% is the Kalman gain term

clear; clc; % figure(3); clf; figure(4); clf; figure(5); clf;

%data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Dispersion_PMIN_191919_Noise.out');
data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Aggregation_RABOFF_181818_noise.out');
[motors_1s motors_5s motors_10s timeobserved motors_10s_avg] = GetRealMotors_m1(data, +1, 0);


AllObservations = motors_10s; 
AllObservations = AllObservations';

figure(3); plot(timeobserved',AllObservations, 'x')


figure(4);clf;
figure(5);clf;
N = length(AllObservations);



theta = linspace(0,50,100);


VarianceInMeasurement = 50; % variance from measurement error %cm
ProcessNoiseVariance_TemporalConstant  = 0.5; %  cm % if max speed per tick is 0.5 cm, the variance can be +/- 0.25cm
%% ProcessNoiseVariance_TemporalConstant  at 0.005 give results similar to using a moving average over past 100 observations

mu_prior  = 0.5; % prior mean
var_prior = 1; % prior variance

total_observations = 0;

for (n=2:length(AllObservations));
    
    
    mu_predicted   = mu_prior;
    %var_predicted  = var_prior + ProcessNoiseVariance_TemporalConstant * 1;
    var_predicted  = var_prior + ProcessNoiseVariance_TemporalConstant * (timeobserved(n) - timeobserved(n-1));
    
    
    
    mu_corrected  = ((mu_predicted / var_predicted) + (AllObservations(n) / VarianceInMeasurement)) / (1/var_predicted + 1/VarianceInMeasurement);    
    var_corrected = 1 / (1/var_predicted + 1/VarianceInMeasurement);
    
    mu_prior  = mu_corrected;
    var_prior = var_corrected;
    
    
    Y_posterior = normpdf(theta,mu_corrected,sqrt(var_corrected));
    
    
%     figure(4); plot(theta,Y_posterior,'r','LineWidth',3); xlabel('theta'); ylabel('P(h|m)')
%     
%     figure(5); hold on;
%     plot(timeobserved(n), mu_prior,'k.','MarkerSize',5); 
%     plot(timeobserved(n), var_prior,'R.','MarkerSize',5), axis([1 5000 0 50]); xlabel('time'); ylabel('E(h|m) in black, Var(h|m) in red ')
%     hold off
    
    kf_data(n,[1:3]) = [timeobserved(n), mu_prior,var_prior];
    
end

    figure(5); hold on;
    plot(timeobserved',AllObservations, '.-')
    plot(kf_data(:,1), kf_data(:,2),'rx-'); 
    plot(kf_data(:,1), kf_data(:,3),'kx-','MarkerSize',3), axis([1 5000 0 50]); xlabel('time'); ylabel('E(h|m) in black, Var(h|m) in red ')
    
    l = line([0, 5000], [7.5, 7.5])
    set(l,'Color',[0,0,0],'LineStyle','--')
    hold off
