function [quadmotors] = GetBooleanMotors_sm2(data, robottype, normalrobotid)

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


% X = 1  when Sensor = False AND Motor = False 
% X = 2  when Sensor = False AND Motor = True
% X = 3  when Sensor = True AND Motor = False 
% X = 4  when Sensor = True AND Motor = True

SENSING_DIST_THRESHOLD = 10; % in cm
data = data (data(:,8) < SENSING_DIST_THRESHOLD  ,:);

DISTANCE_MOVED_THRESHOLD = 2.5; % in cm
ANGULAR_ACCELERATION_THRESHOLD = 0.15; % proportion in range [-1 to 1]

INDEX_FIVESEC_DIST = 5;
INDEX_ANGULAR_ACCE = 7;  

for i = 1:length(data)
    
    if (data(i, INDEX_FIVESEC_DIST) > DISTANCE_MOVED_THRESHOLD) && (abs(data(i, INDEX_ANGULAR_ACCE)) > ANGULAR_ACCELERATION_THRESHOLD)
        quadmotors(i) = 4;
    elseif (data(i, INDEX_FIVESEC_DIST) > DISTANCE_MOVED_THRESHOLD) && (abs(data(i, INDEX_ANGULAR_ACCE)) <= ANGULAR_ACCELERATION_THRESHOLD)
        quadmotors(i) = 3;
    elseif (data(i, INDEX_FIVESEC_DIST) <= DISTANCE_MOVED_THRESHOLD) && (abs(data(i, INDEX_ANGULAR_ACCE)) > ANGULAR_ACCELERATION_THRESHOLD)
        quadmotors(i) = 2;
    else
        quadmotors(i) = 1;
    end    
end

end