function [real_sensors_NumNbrs_close real_sensors_NumNbrs_far real_sensors_CoM_close real_sensors_CoM_far timeobserved] = GetRealSensors_s1(data, robottype, normalrobotid)

% robottype = -1 % look for abnormally behaving robot
% robottype = +1 % look for normally behaving robot

if robottype == -1
    focalid = 15;
else
    focalid = normalrobotid;
end

% When robottype is -1
% Lets start with looking at observations from the robot that has observed robot 15 the most number of times. 

% When robotype is +1
% Lets start with looking for observations from the robot that has observed robot XXX (focalid) the most number of times. 

numobservations = zeros(20,1);
for observer_robot_id = 0:19
   numobservations(observer_robot_id+1) = length(intersect(find(data(:,2)==focalid), find(data(:,3)==observer_robot_id))); 
end

best_observer = find(numobservations == max(numobservations)) - 1;
data = data(intersect(find(data(:,2)==focalid), find(data(:,3)==best_observer)),:); 

data_backup = data;

INDEX_COM_CLOSE       = 9;
INDEX_COM_FAR         = 10;
INDEX_NUMNBRS_CLOSE   = 11;
INDEX_NUMNBRS_FAR     = 12;

% MAX_CoM_CLOSE        = 15;  %% 15 cm
% 
% MIN_CoM_FAR          = 15;  %% 15 cm 
% MAX_CoM_FAR          = 30;  %% 30 cm
% 
% 
% MAX_NumNbrs_CLOSE    = 5; 
% MAX_NumNbrs_FAR      = 5; 



%% Looking at the time the observations were made
timeobserved = data(:, 1);


%% Looking at our sensors output feature - COM of neighbours in close range
real_sensors_CoM_close  =   data(:,INDEX_COM_CLOSE);
%real_sensors_CoM_close  =  real_sensors_CoM_close ./ MAX_CoM_CLOSE;



%% Looking at our sensors output feature - COM of neighbours in far range
data = data_backup;

% normalise [15, 30] cm to [ 0, 1]
real_sensors_CoM_far   =  data(:,INDEX_COM_FAR);
%real_sensors_CoM_far   =  (real_sensors_CoM_far - MIN_CoM_FAR) ./ (MAX_CoM_FAR - MIN_CoM_FAR);

%real_sensors_CoM_far(real_sensors_CoM_far > 1 ) = 1;
%real_sensors_CoM_far(real_sensors_CoM_far < 0 ) = 0;


%% Looking at our sensors output feature - number of neighbours in close range
data = data_backup;

real_sensors_NumNbrs_close    =  data(:,INDEX_NUMNBRS_CLOSE);
%real_sensors_NumNbrs_close    =  real_sensors_NumNbrs_close ./ MAX_NumNbrs_CLOSE;
%real_sensors_NumNbrs_close(real_sensors_NumNbrs_close > 1 ) = 1;


%% Looking at our sensors output feature - number of neighbours in far range
data = data_backup;

real_sensors_NumNbrs_far    =  data(:,INDEX_NUMNBRS_FAR);
%real_sensors_NumNbrs_far    =  real_sensors_NumNbrs_far ./ MAX_NumNbrs_FAR;
%real_sensors_NumNbrs_far(real_sensors_NumNbrs_far > 1 ) = 1;

end