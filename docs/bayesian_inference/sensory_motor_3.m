%% Bayesian statistical inference of sensory motor interactions

%% Our model is an HMM because our observations each trial may not be independent and identitically distributed.
%% Since X(t) is binary, what we have here is a binary Bayesian filter.
%% Ref: Hongbin Ma; Dong Wang; Hongsheng Qi; Mengyin Fu, "An approach of Bayesian filtering for stochastic Boolean dynamic systems," Control and Decision Conference (2014 CCDC), The 26th Chinese , vol., no., pp.4335,4340, May 31 2014-June 2 2014

%% Please also see Braga-Neto, Ulisses. "Optimal state estimation for Boolean dynamical systems." Signals, Systems and Computers (ASILOMAR), 2011 Conference Record of the Forty Fifth Asilomar Conference on. IEEE, 2011.

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

% X = True  when Sensor = True AND Motor = True 
% X = False when Sensor = True AND Motor = False
% So at step t, when Sensor is True, we observe X. 


% Prior probability. Our current belief of the SM interaction of the robot.
% At the start, we don't know. So it follows a uniform distribution. 
% P(Theta = True) = 0.5; P(Theta = False) = 0.5;
% Lets call this P ( Theta(0) )


% We need a probability state transition matrix for our hidden variable
% Theta. Lets call this P_Theta_Theta
% P(Theta(t)=T | Theta(t-1)=T) = 0.99 || 
% P(Theta(t)=F | Theta(t-1)=T) = 0.01

% P(Theta(t)=T | Theta(t-1)=F) = 0.01 || 
% P(Theta(t)=F | Theta(t-1)=F) = 0.99


% We also need a state-measurement probability transition matrix which says
% what is the probability that the measurement is X(t) when the state is
% Theta(t). Lets call this P_X_Theta
% P ( X(t) = T | Theta(t) = T ) = 0.8 || P ( X(t) = F | Theta(t) = T ) = 0.2 
% P ( X(t) = T | Theta(t) = F ) = 0.5 || P ( X(t) = F | Theta(t) = F ) = 0.5 


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

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Aggregation_STOP.out');
%AllObservations = GetBooleanMotors_sm1(data, +1, 7);


normalised_motor_threshold = 0.1;
[real_sensor(:,1) motors motors_nosensors motors_irrespsensors] = GetRealSensorsMotors_sm3(data, -1, 0);
%% normalised motor values in presence of nbr robots
motors(motors >= normalised_motor_threshold) = 1;
motors(motors <  normalised_motor_threshold) = 0;
AllObservations = motors;



Prior = [0.5 0.5];

alpha = 0.99999999;
StateTransMatrix        = [alpha   1-alpha
                           1-alpha  alpha];  

StateMeasurementMatrix  = [0.2  0.8
                           0.50  0.50];  

for timeindex = 1:length(AllObservations)                
    
    Observation = AllObservations(timeindex);  % X = 0 or 1
    
    if Observation == 0
        Observation = 2; %% mapping to 
    end

    Prediction =  Prior * StateTransMatrix';

    Correction = (StateMeasurementMatrix(:,Observation)  .* Prediction') / (StateMeasurementMatrix(:,Observation)' * Prediction');

    Prior = Correction'; % new prior
    
    figure(fg); hold on;
    plot(timeindex,Prior(1),'k.');axis([0 length(AllObservations) 0 1 ])    % Expectation = Prior(1) * 1 + Prior(2) * 0
end


