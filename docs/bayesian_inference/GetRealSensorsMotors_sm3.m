function [real_sensors real_motors real_motors_NoNbrsPresent real_motors_IrrespectiveNbrsPresent timeobserved timeobserved_NoNbrsPresent timeobserved_IrrespectiveNbrsPresent] = GetRealSensorsMotors_sm3(data, robottype, normalrobotid)

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


INDEX_FIVESEC_DIST     = 5;
INDEX_ANGULAR_ACCE     = 7;  
INDEX_DIST_NEAREST_NBR = 8;

MAX_DIST_IN_FIVESEC    = 25; 


%% Looking at our motor output feature, when sensor information is present (i.e. nearest nbr is less than 30 cm away)
MAX_SENSING_DIST_THRESHOLD = 30; % in cm
MIN_SENSING_DIST_THRESHOLD = 10; % in cm
data = data (data(:, INDEX_DIST_NEAREST_NBR ) < MAX_SENSING_DIST_THRESHOLD  ,:);
real_sensors = ( data(:, INDEX_DIST_NEAREST_NBR) - MIN_SENSING_DIST_THRESHOLD )/ (MAX_SENSING_DIST_THRESHOLD - MIN_SENSING_DIST_THRESHOLD);
real_sensors(real_sensors > 1) = 1;
real_sensors(real_sensors < 0) = 0;

angularacceleration = abs(data(:, INDEX_ANGULAR_ACCE )); % already in range 0 to 1
disttravelled       = data(:,INDEX_FIVESEC_DIST)/MAX_DIST_IN_FIVESEC;
disttravelled(disttravelled > 1) = 1;
disttravelled(disttravelled < 0) = 0;
real_motors = angularacceleration .* disttravelled;

timeobserved = data(:, 1);


%% Looking at our motor output feature, when sensor information is NOT present 
data = data_backup;

MAX_SENSING_DIST_THRESHOLD = 100; % in cm
MIN_SENSING_DIST_THRESHOLD = 30; % in cm
data = data (data(:, INDEX_DIST_NEAREST_NBR ) > MIN_SENSING_DIST_THRESHOLD  ,:);
real_sensors = ( data(:, INDEX_DIST_NEAREST_NBR) - MIN_SENSING_DIST_THRESHOLD )/ (MAX_SENSING_DIST_THRESHOLD - MIN_SENSING_DIST_THRESHOLD);
real_sensors(real_sensors > 1) = 1;
real_sensors(real_sensors < 0) = 0;

angularacceleration = abs(data(:, INDEX_ANGULAR_ACCE )); % already in range 0 to 1
disttravelled       = data(:,INDEX_FIVESEC_DIST)/MAX_DIST_IN_FIVESEC;
disttravelled(disttravelled > 1) = 1;
disttravelled(disttravelled < 0) = 0;
real_motors_NoNbrsPresent = angularacceleration .* disttravelled;

timeobserved_NoNbrsPresent = data(:, 1);


%% Looking at our motor output feature, irrespective of sensor information
data = data_backup;

MAX_SENSING_DIST_THRESHOLD = 100; % in cm
MIN_SENSING_DIST_THRESHOLD = 10; % in cm
real_sensors = ( data(:, INDEX_DIST_NEAREST_NBR) - MIN_SENSING_DIST_THRESHOLD )/ (MAX_SENSING_DIST_THRESHOLD - MIN_SENSING_DIST_THRESHOLD);
real_sensors(real_sensors > 1) = 1;
real_sensors(real_sensors < 0) = 0;

angularacceleration = abs(data(:, INDEX_ANGULAR_ACCE )); % already in range 0 to 1
disttravelled       = data(:,INDEX_FIVESEC_DIST)/MAX_DIST_IN_FIVESEC;
disttravelled(disttravelled > 1) = 1;
disttravelled(disttravelled < 0) = 0;
real_motors_IrrespectiveNbrsPresent = angularacceleration .* disttravelled;

timeobserved_IrrespectiveNbrsPresent = data(:, 1);

end