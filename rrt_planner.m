ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

load exampleMaps
map = occupancyMap(simpleMap,10);

sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];

planner = plannerRRTStar(ss,sv);
planner.MaxConnectionDistance = 0.15;
planner.MaxNumTreeNodes = 1e3;
planner.MaxIterations = 1e3;
planner.GoalReachedFcn = @checkIfReached;

start = [0.5,0.5,0];
goal = [2.5,0.2,0];

%rng(100,'twister'); % for repeatable result
[pthObj,solnInfo] = planner.plan(start,goal);

%video = VideoWriter('rrtStar.avi'); %create the video object
%open(video); %open the file for writing
e = 0;
%points = length(pthObj.States);
for i = 1:pthObj.NumStates
    if i == 1
        prev_state = start;
    end
    
    % Plot map
    figure(1); map.show; hold on;
    
    sub = "Energy cost " + num2str(e);
    title(sub)
    
    % Plot generated tree data
    figure(1); plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-',...
        'Color',[0.522 0.0784 0.357]); hold on;
    
    % Plot path found
    figure(1); plot(pthObj.States(1:i,1), pthObj.States(1:i,2),'-',...
        'Color',[0.235 0.8 0.047], 'LineWidth',2); hold on % draw path
    
    % Energy_Cost
    if i~=1
        next_state = pthObj.States(i,:);
        energy = min_energy(prev_state, next_state);
        e = e + energy;
        prev_state = next_state;
    end
    
    % Start point in green, end point in red
    figure(1); plot(start(1), start(2), '.', 'Color',[0.047 0.137 0.8], 'MarkerSize',20); hold on;
    figure(1); plot(goal(1), goal(2), '.', 'Color',[0.8 0.047 0.047], 'MarkerSize',20); hold on;

    % Create & save gif
    plt_name = strcat('path', int2str(i), '.png');
    %pause(0.5)
    frame = getframe(figure(1));
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256);
    if i==1
        imwrite(imind,cm,'rrtPath.gif','gif', 'Loopcount',inf);
    else
        imwrite(imind,cm,'rrtPath.gif','gif','WriteMode','append');
    end
    
    
    % Try to make video
    %writeVideo(video,im); %write the image to file
end
%close(video); %close the file