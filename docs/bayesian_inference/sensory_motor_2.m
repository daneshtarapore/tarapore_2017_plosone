%% Bayesian statistical inference of sensory motor interactions

%% Our model.

% The boolean random variable describing the SM interaction of robot (Latent variable depending on state of robot
% and controller). Lets call this Theta.

% Observed SM interaction of robot. This is what we actually observe of the
% robot. Lets call this X

% The event we are interested in is how does the robot Motor respond to
% Sensor.

% X = 1  when Sensor = False AND Motor = False 
% X = 2  when Sensor = False AND Motor = True
% X = 3  when Sensor = True AND Motor = False 
% X = 4  when Sensor = True AND Motor = True
% So at step t, we observe X. 


% Prior probability. Our current belief of the SM interaction of the robot.
% At the start, we don't know. So it follows a uniform distribution. 
% P(Theta = True) = 0.5; P(Theta = False) = 0.5;


% Likelyhood - The conditional probability table. This is our model of what
% kind of observations X you are going to see for different values of
% Theta.
% P(X = 1 | Theta = True) = 0.33  || P(X = 2 | Theta = True) = 0.33 || 
% P(X = 3 | Theta = True) = 0.01 || P(X = 4 | Theta = True) = 0.33

% P(X = 1 | Theta = False) = 0.25  || P(X = 2 | Theta = False) = 0.25 || 
% P(X = 3 | Theta = False) = 0.25 || P(X = 4 | Theta = False) = 0.25

% Bayesian inference
% P(Theta | X) = P(X | Theta) * P (Theta) / P (X)
% So for each observation X (whether True or False) we update the
% whole posterior probability i.e. both P(Theta = T | X) and P(Theta = F | X)

clear; fg = figure;

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Dispersion_STOP.out');
AllObservations = GetBooleanMotors_sm2(data, +1, 18);

Prior = [0.5 0.5];

Likelyhood = [0.25 0.25 0.15 0.35
              0.25 0.25 0.25 0.25];  

          
for time = 1:length(AllObservations)          

    Observation = AllObservations(time);  % X = 1 or 2 or 3 or 4

    %P(Theta = T | X) 
    p1_tmp = Prior(1) * Likelyhood(1, Observation);

    %P(Theta = F | X) 
    p2_tmp = Prior(2) * Likelyhood(2, Observation);    

    % Computing P(X) is difficult. But we know that P(Theta = T | X) + P(Theta = F| X) = 1 
    p1 = p1_tmp / (p1_tmp + p2_tmp);
    p2 = p2_tmp / (p1_tmp + p2_tmp);

    Prior = [p1 p2]; % new prior
    
    figure(fg); hold on;
    plot(time,Prior(1),'k.');axis([0 length(AllObservations) 0 1 ])    % Expectation = Prior(1) * 1 + Prior(2) * 0
end
