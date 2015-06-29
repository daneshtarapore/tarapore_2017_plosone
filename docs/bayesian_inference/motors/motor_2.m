%Recursive bayesian estimation of the robots motor outputs
% Using a uni-variate gaussian distribution
% Modeling prior as a Gaussian distribution, so that as the prior,
% posterior and likeihood are Gaussian distributions, a closed form
% expression of the posterior hyperparameters can be obtained.


clear; clc; close all

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Aggregation_CIRCLE.out');
[motors_1s motors_5s motors_10s] = GetRealMotors_m1(data, +1, 18);

AllObservations = motors_10s; 
AllObservations = AllObservations';


theta = linspace(0,1,100);


var_likelihood=[1]; % variance of the likelihood Gaussian distributions
mu_prior  = 0.5; % prior mean
var_prior = 4; % prior variance

total_observations = 0;

for (n=1:length(AllObservations));
    
    total_observations = total_observations + AllObservations(n);
    
    mu_posterior = ((mu_prior / var_prior) + (total_observations / var_likelihood)) / (1/var_prior + n/var_likelihood);    
    var_posterior = 1 / (1/var_prior + n/var_likelihood);
    
    
    Y_posterior = normpdf(theta,mu_posterior,sqrt(var_posterior));
    
    
    figure(1); plot(theta,Y_posterior,'r','LineWidth',3); xlabel('time'); ylabel('P(h|m)')
    
    figure(2); hold on;
    plot(n,mu_posterior,'k.','MarkerSize',5), axis([1 length(AllObservations) 0 1]); xlabel('time'); ylabel('E(m|s)')
    hold off
    
end