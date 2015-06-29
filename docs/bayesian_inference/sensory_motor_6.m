%Recursive bayesian estimation of the robots sensory-motor interactions
% Using a uni-variate gaussian distribution


clear

data            = load('/home/danesh/argos3-foraging/docs/bayesian_inference/dataforBI_Dispersion_RNDWK.out');
[x(:,1) x(:,2) motors_nosensors motors_irrespsensors] = GetRealSensorsMotors_sm3(data, -1, 9);
x = x';


figure(1);clf;
figure(2);clf;
N = length(x);

%s=[3;5];  % where the quail is hiding

%quail squawks N times, and for each squawk, the ninja hears the quail at
%some location. The error in estimation of where the qual is will be
%defined as gaussian around where the quail actually is with a standard
%deviation of 2.

%n=2*randn(2,N); % Create vector of iterative squawks with a standard deviation of 2.

%center the gaussian random swarks around the point where the quail
%actually is. and plot
figure(1);

%make the plot prettier
%h=plot(s(1),s(2),'r.');  % Plot where the quail actually is
%set(h,'markersize',40,'linewidth',3); % make pretty
axis([0,1,0,1]);  % make pretty
hold off;
hold on
for i=1:N
   plot(0.5, x(2,i),'k.','markersize',10);
   %pause
end;
xlabel('Sensor'); ylabel('Motor')





%now, we do Ninja bayes!

%define the locations the quail can be at
Sa=[0:0.05:1];


%no bias, no prior knowledge, uniform distribution
L=length(Sa);
Pr=ones(L,1); % Initialize all one --> uniform prior
Po=ones(L,1); %duplicate for iterative update

%bias the prior toward the quail
%figure(1);mesh(centered_prior), axis([0 40 0 40 0 0.015])
%Po = centered_prior;
%Pr = centered_prior;

%bias the prior away from the quail
%figure(1);mesh(off_centered_prior), axis([0 40 0 40 0 0.015])
%Po = off_centered_prior;
%Pr = off_centered_prior;

Pr=Pr/sum(sum(Pr)); % Turn the prior into a pmf by dividing by the sum.
Po=Po/sum(sum(Po)); % Each value is now 1/(number of states), so it all sums to one.
figure(1);clf;plot(1:length(Po),Po); axis([0 20 0 0.5])






%%iterative bayes

[a]=find(Po==max(max(Po)));  % Pull out the indices at which Po achieves its max to start.
sest=[Sa(a)];  % The best estimate of the true state to start.
figure(1);
clf
figure(2);
clf
plot(1,sest(1)); hold on;
%line([1,N],[s(1),s(1)]); % Draw a line at the location of the x dimension.


K=[4]; % covariance matrix for making a 2-D gaussian
%K=[4.16,1.6;1.6,4.16]; % covariance matrix for making a 2-D gaussian
for (n=2:length(x));
    Pr=Po; %store the posterior to the prior.
    m=0*Pr;   
    %likelihood
    % look at each location, assume that the given location is
    % is there quail is, and get the likelihood of the data x(:,n) assuming
    % 2-d gaussian noise
    for (i=1:length(Pr))
           me=[Sa(i)];
           m(i) = 1/(sqrt((2*pi)* K)) * exp(-((x(n)-me)^2)/(2 * K) ); %Compute likelihood      
           m(i) = m(i) * Pr(i); % Combine this likelihood with the prior   
    end;
    Po=m/sum(sum(m)); %normalize this distribution to make it a proper probability distribution.
    figure(1); plot(1:length(Po),Po); axis([0 20 0 0.5]); xlabel('Motors'); ylabel('P(h|m)')  %plot it
    figure(2);
    [a]=find(Po==max(max(Po)));  % Get the peak value; it's most likely location of the Quail.
    sest=[Sa(a)];  %A store the coordinates of this location in the bushes
    plot(n,sest(1),'k.'); ylabel('Infered Motor in SM interaction with highest posterior probability'); axis([0 N 0 1 ])
    %pause
end;  
hold off;