clear; close all; clc;

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Homing_STOP.out');
%AllObservations = GetBooleanMotors_sm1(data, +1, 9);


normalised_motor_threshold = 0.1;
[x(:,1) motors motors_nosensors motors_irrespsensors timeobserved timeobserved_NoNbrsPresent timeobserved_IrrespectiveNbrsPresent] = GetRealSensorsMotors_sm3(data, -1, 18);

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


% Prior parameters
a = 1;
b = 1;

theta = linspace(0,1,100);
Y_prior = betapdf(theta,a,b);
    
for time = 1:length(AllObservations)
    
    % Data
    N = time;
    X = sum(AllObservations(1:time));
    
    
    Y_likelihood = nchoosek(N,X)*(theta.^X).*(1-theta).^(N-X);
    Y_posterior = betapdf(theta,a+X,b+N-X);
    
    h = figure(2);
    subplot(4,1,1),
    plot(theta,Y_prior,'LineWidth',3)
    title('Prior','FontSize',20)
    set(gca,'FontSize',20)
    ylabel('pdf')
    
    subplot(4,1,2),
    plot(theta,Y_likelihood,'m','LineWidth',3)
    set(gca,'FontSize',20)
    title('Likelihood','FontSize',20)
    ylabel('likelihood')
    
    subplot(4,1,3),
    plot(theta,Y_posterior,'r','LineWidth',3)
    title('Posterior','FontSize', 20)
    ylabel('pdf')
    %set(h,'Position',[1000 150 900 900])
    set(gca,'FontSize',20)
    xlabel('Theta')
    
    subplot(4,1,4),   
    hold on
    %plot(time,(a+X)/(b+N-X),'k.','MarkerSize',5), axis([1 length(AllObservations) 0 1])
    plot(time_vector(time),(a+X)/(a+b+N),'k.','MarkerSize',5), axis([0 5000 0 1])
    plot(time_vector(time),((a+X)*(a+b+N))/(((a+X)+(a+b+N))*((a+X)+(a+b+N))*((a+X)+(a+b+N)+1)),'r.','MarkerSize',5)
    title('Expected value ','FontSize', 20)
    ylabel('E[.] black Var[.] red')
    set(h,'Position',[1000 150 900 900])
    set(gca,'FontSize',20)
    xlabel('time')
    
    
    %Y_prior = Y_posterior;
    %a = a+X;
    %b = b+N-X;
    
    
    %pause
end