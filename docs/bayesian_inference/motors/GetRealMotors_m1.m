function [real_motors_1s real_motors_5s real_motors_10s time_vector real_motors_10s_avg] = GetRealMotors_m1(data, robottype, normalrobotid)

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

INDEX_ONESEC_DIST     = 4;
INDEX_FIVESEC_DIST    = 5;
INDEX_TENSEC_DIST     = 6;

INDEX_TENSEC_DIST_AVG     = 7;

MAX_DIST_IN_ONESEC    = 1; %5; 
MAX_DIST_IN_FIVESEC   = 1; %25; 
MAX_DIST_IN_TENSEC    = 1; %50; 


%% Looking at our motor output feature in 1 second interval
disttravelled  =   data(:,INDEX_ONESEC_DIST);
real_motors_1s  =  disttravelled ./ MAX_DIST_IN_ONESEC;



%% Looking at our motor output feature in 5 second interval
data = data_backup;

disttravelled   =  data(:,INDEX_FIVESEC_DIST);
real_motors_5s  =  disttravelled ./ MAX_DIST_IN_FIVESEC;


%% Looking at our motor output feature in 10 second interval
data = data_backup;

disttravelled    =  data(:,INDEX_TENSEC_DIST);
real_motors_10s  =  disttravelled ./ MAX_DIST_IN_TENSEC;


%% Looking at our motor output feature averaged in 10 second interval
data = data_backup;

disttravelled    =  data(:,INDEX_TENSEC_DIST_AVG);
real_motors_10s_avg =  disttravelled ./ MAX_DIST_IN_TENSEC;


time_vector = data(:, 1);

end