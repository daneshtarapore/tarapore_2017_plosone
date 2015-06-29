%Recursive bayesian estimation of the robots odometric output - distance
%traversed in last 10s
% using a Kalman filter approach


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


clear; close all; clc;

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Dispersion_STOP.out');
%AllObservations = GetBooleanMotors_sm1(data, +1, 9);


normalised_motor_threshold = 0.1;
[x(:,1) motors motors_nosensors motors_irrespsensors timeobserved timeobserved_NoNbrsPresent timeobserved_IrrespectiveNbrsPresent] = GetRealSensorsMotors_sm3(data, +1, 18);

%% normalised motor values in presence of nbr robots
motors(motors >= normalised_motor_threshold) = 1;
motors(motors <  normalised_motor_threshold) = 0;
AllObservations = motors;
time_vector = timeobserved;

%% normalised motor values in absence of nbr robots
% motors_nosensors(motors_nosensors >= normalised_motor_threshold) = 1;
% motors_nosensors(motors_nosensors <  normalised_motor_threshold) = 0;
% AllObservations = motors_nosensors;

%% normalised motor values in irrespective of nbr robots
% motors_irrespsensors(motors_irrespsensors >= normalised_motor_threshold) = 1;
% motors_irrespsensors(motors_irrespsensors <  normalised_motor_threshold) = 0;
% AllObservations = motors_irrespsensors;





theta = linspace(0,1,100);


VarianceInMeasurement = 1; % variance from measurement error %cm
ProcessNoiseVariance_TemporalConstant  = 0.0001; %  cm % if max speed per tick is 0.5 cm, the variance can be +/- 0.25cm

mu_prior  = 0.5; % prior mean
var_prior = 1; % prior variance

total_observations = 0;

for (n=2:length(AllObservations));
    
    
    mu_predicted   = mu_prior;
    %var_predicted  = var_prior + ProcessNoiseVariance_TemporalConstant * 1;
    var_predicted  = var_prior + ProcessNoiseVariance_TemporalConstant * (timeobserved(n) - timeobserved(n-1));
    
    NewObservation = AllObservations(n) %% Our estimate of the time since the last observation of motor interaction in the presence (absence, irrespective) of sensor input
    
    mu_corrected  = ((mu_predicted / var_predicted) + (NewObservation / VarianceInMeasurement)) / (1/var_predicted + 1/VarianceInMeasurement);    
    var_corrected = 1 / (1/var_predicted + 1/VarianceInMeasurement);
    
    mu_prior  = mu_corrected;
    var_prior = var_corrected;
    
    
    Y_posterior = normpdf(theta,mu_corrected,sqrt(var_corrected));
    
    
    figure(4); plot(theta,Y_posterior,'r','LineWidth',3); xlabel('theta'); ylabel('P(h|m)')
    
    figure(5); hold on;
    plot(timeobserved(n), mu_prior,'k.','MarkerSize',5); 
    plot(timeobserved(n), var_prior,'R.','MarkerSize',5), axis([1 5000 0 1]); xlabel('time'); ylabel('E(h|m) in black, Var(h|m) in red ')
    hold off
    
end

