function isReached = checkIfReached(planner,currentState,goalState)
    dist = (currentState(1)-goalState(1))^2+(currentState(2)-goalState(2))^2;
    if sqrt(dist) < 0.005
        isReached = true;
    else
        isReached = false;
    end
end