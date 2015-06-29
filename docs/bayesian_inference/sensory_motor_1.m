%% Bayesian statistical inference of sensory motor interactions

%% Our model.

% The boolean random variable describing the SM interaction of robot (Latent variable depending on state of robot
% and controller). Lets call this Theta.

% Observed SM interaction of robot. This is what we actually observe of the
% robot. Lets call this X

% The event we are interested in is how does the robot Motor respond iff
% Sensor is True. We can not say anything about SM interaction when Sensor
% is absent (i.e., the robot has no neighbours).

% X = True  when Sensor = True AND Motor = True 
% X = False when Sensor = True AND Motor = False
% So at step t, when Sensor is True, we observe X. 


% Prior probability. Our current belief of the SM interaction of the robot.
% At the start, we don't know. So it follows a uniform distribution. 
% P(Theta = True) = 0.5; P(Theta = False) = 0.5;


% Likelyhood - The conditional probability table. This is our model of what
% kind of observations X you are going to see for different values of
% Theta.
% P(X = True | Theta = True) = 0.8  || P(X = False | Theta = True) = 0.2 (we don't want this to be 0 as this would make P(Theta=False) irresvible - to be tested.)
% P(X = True | Theta = False) = 0.5 || P(X = False | Theta = False) = 0.5 

% Bayesian inference
% P(Theta | X) = P(X | Theta) * P (Theta) / P (X)
% So for each observation X (whether True or False) we update the
% whole posterior probability i.e. both P(Theta = T | X) and P(Theta = F | X)

clear; fg= figure;

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Dispersion_STOP.out');
AllObservations = GetBooleanMotors_sm1(data, +1, 1);


Prior = [0.5 0.5];

alpha = 0.6;

Likelyhood = [alpha 1-alpha
              0.5  0.5];


for time = 1:length(AllObservations)
    
    %Observation = 0;  % X = 0 or 1
    Observation = AllObservations(time);
    
    if Observation == 0
        Observation = 2; %% mapping to false
    end
    
    %P(Theta = T | X)
    p1_tmp = Prior(1) * Likelyhood(1, Observation);
    
    %P(Theta = F | X)
    p2_tmp = Prior(2) * Likelyhood(2, Observation);
    
    % Computing P(X) is difficult. But we know that P(Theta = T | X) + P(Theta = F| X) = 1
    p1 = p1_tmp / (p1_tmp + p2_tmp);
    p2 = p2_tmp / (p1_tmp + p2_tmp);
    
    Observation
    Prior = [p1 p2] % new prior
    
    figure(fg); hold on;
    plot(time,Prior(1),'k.');axis([0 length(AllObservations) 0 1 ])    % Expectation = Prior(1) * 1 + Prior(2) * 0
    
end
