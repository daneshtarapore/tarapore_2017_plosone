%Recursive bayesian estimation of the robots sensory inputs
% Using a uni-variate gaussian distribution
% Modeling prior as a Gaussian distribution, so that as the prior,
% posterior and likeihood are Gaussian distributions, a closed form
% expression of the posterior hyperparameters can be obtained.


clear; clc; close all

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/sensors/nbrsdataforBI_Dispersion_RANDOMWALK.out');
[real_sensors_NumNbrs_close real_sensors_NumNbrs_far real_sensors_CoM_close real_sensors_CoM_far ] = GetRealSensors_s1(data, +1, 8);


AllObservations = real_sensors_CoM_far; 
%AllObservations = real_sensors_CoM_far; 
AllObservations = AllObservations';


figure(1);clf;
figure(2);clf;
N = length(AllObservations);



theta = linspace(0,1,100);


var_likelihood=[1]; % variance of the likelihood Gaussian distributions
mu_prior  = 0.5; % prior mean
var_prior = 4; % prior variance

total_observations = 0;

for (n=1:length(AllObservations));
    
    total_observations = total_observations + AllObservations(n);
    
    %mu_posterior = ((mu_prior / var_prior) + (total_observations / var_likelihood)) / (1/var_prior + n/var_likelihood);    
    %var_posterior = 1 / (1/var_prior + n/var_likelihood);
    
    mu_posterior = ((mu_prior / var_prior) + (AllObservations(n) / var_likelihood)) / (1/var_prior + 1/var_likelihood);    
    var_posterior = 1 / (1/var_prior + 1/var_likelihood);
    
    mu_prior  = mu_posterior;
    var_prior = var_posterior;
    
    
    Y_posterior = normpdf(theta,mu_posterior,sqrt(var_posterior));
    
    
    figure(1); plot(theta,Y_posterior,'r','LineWidth',3); xlabel('time'); ylabel('P(h|s)')
    
    figure(2); hold on;
    plot(n,mu_prior,'k.','MarkerSize',5); 
    plot(n,var_prior,'R.','MarkerSize',5), axis([1 length(AllObservations) 0 1]); xlabel('time'); ylabel('E(h|s) in black, Var(h|s) in red ')
    hold off
    
end

