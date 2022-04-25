%function done = rrt_planner(waypoints)
% Import robot_parameters
robot_parameters

% Import simpleMap, complexMap and ternaryMap
load exampleMaps.mat

% Load simpleMap (Test#1)
map = binaryOccupancyMap(simpleMap,2);
show(map)

% To properly account for obstacle avoidance for the robot, inflate the 
% map by robot dimensions
robot_radius = L;
inflatedMap = copy(map);
inflate(inflatedMap,robot_radius);

% Construct PRM & set params
prm = mobileRobotPRM;
prm.Map = inflatedMap;
prm.NumNodes = 250;
prm.ConnectionDistance = 2.5;
startLocation = [2 1];
endLocation = [12 10];
path = findpath(prm, startLocation, endLocation)
%path(1,:)

pts_len = length(path);
for i=1:pts_len
    endLocation = path(i,:);
    path1 = findpath(prm, startLocation, endLocation);
    show(prm)
%    exportgraphics(f, strcat('path.png', int2str(i)), 'Resolution', 300)
%    saveas(prm, strcat('path.png', int2str(i)))
    startLocation = path(i,:);
end