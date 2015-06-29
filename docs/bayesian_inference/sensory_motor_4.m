%% Bayesian statistical inference of sensory motor interactions

%% Our model is an HMM because our observations each trial may not be independent and identitically distributed.
%% Since X(t) is binary, what we have here is a binary Bayesian filter.
%% Ref: Hongbin Ma; Dong Wang; Hongsheng Qi; Mengyin Fu, "An approach of Bayesian filtering for stochastic Boolean dynamic systems," Control and Decision Conference (2014 CCDC), The 26th Chinese , vol., no., pp.4335,4340, May 31 2014-June 2 2014

% Theta(t-1) -> Theta(t)
%       |          |  
%       V          V
%    X(t-1) ->    X(t) 
 

% The boolean random variable describing the SM interaction of robot (Latent variable depending on state of robot
% and controller). Lets call this Theta.

% Observed SM interaction of robot. This is what we actually observe of the
% robot. Lets call this X

% The event we are interested in is how does the robot Motor respond iff
% Sensor is True. We can not say anything about SM interaction when Sensor
% is absent (i.e., the robot has no neighbours).


% X = 1  when Sensor = False AND Motor = False 
% X = 2  when Sensor = False AND Motor = True
% X = 3  when Sensor = True AND Motor = False 
% X = 4  when Sensor = True AND Motor = True
% So at step t, we observe X. 


% Prior probability. Our current belief of the SM interaction of the robot.
% At the start, we don't know. So it follows a uniform distribution. 
% P(Theta(0) = True) = 0.5; P(Theta(0) = False) = 0.5;
% Lets call this P ( Theta(0) )


% We need a probability state transition matrix for our hidden variable
% Theta. Lets call this P_Theta_Theta. Since a functioning robot has a very
% low probability to stop functioning, we set high
% probability values P(Theta(t)=T | Theta(t-1)=T) and vice versa.
% P(Theta(t)=T | Theta(t-1)=T) = 0.99 || 
% P(Theta(t)=F | Theta(t-1)=T) = 0.01

% P(Theta(t)=T | Theta(t-1)=F) = 0.01 || 
% P(Theta(t)=F | Theta(t-1)=F) = 0.99


% We also need a state-measurement probability transition matrix which says
% what is the probability that the measurement is X(t) when the state is
% Theta(t). Lets call this P_X_Theta
% P(X(t) = 1 | Theta(t) = True) = 0.33  || P(X(t) = 2 | Theta(t) = True) = 0.33 || 
% P(X(t) = 3 | Theta(t) = True) = 0.01 || P(X(t) = 4 | Theta(t) = True) = 0.33

% P(X(t) = 1 | Theta(t) = False) = 0.25  || P(X(t) = 2 | Theta = False) = 0.25 || 
% P(X(t) = 3 | Theta(t) = False) = 0.25 || P(X(t) = 4 | Theta = False) = 0.25


% Bayesian inference. This consists of a prediction step followed by an update step.
% The prediction step:
% P(Theta(t) | Theta(t-1)) = P_Theta_Theta * P(Theta(t-1) | Theta(t-1)) 

% The update step  - correction based on current observation X(t):
% P(Theta(t) | Theta(t)) =   P_X_Theta( X(t) )  o P(Theta(t-1) | Theta(t-1)) 
%                            -------------------------------------------------------------  
%                            Transpose ( P_X_Theta( X(t) ) )  *   P(Theta(t-1) | Theta(t-1)) 

% Note: o is the hammard product, also called the Schur product or the
% entrywise product.


% So for each observation X(t) (whether True or False) we update the
% whole posterior probability i.e. both P(Theta(t))

clear; fg = figure;

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Homing_STOP.out');
AllObservations = GetBooleanMotors_sm2(data, +1, 18);


Prior = [0.5 0.5];

alpha = 0.99999999;
StateTransMatrix        = [alpha   1-alpha
                           1-alpha  alpha];  


                       
StateMeasurementMatrix = [0.25 0.25 0.15 0.35
                          0.25 0.25 0.25 0.25];  


for timeindex = 1:length(AllObservations)                
    
    Observation = AllObservations(timeindex);  % X = 0 or 1

    Prediction =  Prior * StateTransMatrix';

    Correction = (StateMeasurementMatrix(:,Observation)  .* Prediction') / (StateMeasurementMatrix(:,Observation)' * Prediction');


    Prior = Correction'; % new prior
    
    figure(fg); hold on;
    plot(timeindex,Prior(1),'k.');axis([0 length(AllObservations) 0 1 ])    % Expectation = Prior(1) * 1 + Prior(2) * 0
end


