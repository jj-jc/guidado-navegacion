function isReached = maxErrorPlanner(planner, currentState,goalState)
    isReached = false;
    maxDistance = 0.02;
    if planner.StateSpace.distance(goalState, currentState) < maxDistance
        isReached = true;
    end
end