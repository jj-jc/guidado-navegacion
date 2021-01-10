function [distanceError] = control(ts, trajectory, controller)
    pose3D = apoloGetLocationMRobot('Pioneer3AT');
    pose = [pose3D(1) pose3D(2) pose3D(4)];
    [vl, va] = controller(pose);
    apoloMoveMRobot('Pioneer3AT',[vl, va], ts);
    apoloUpdate();
    distanceToGoal = norm([pose(1) pose(2)] - trajectory(end,:));
    distanceError = distanceToGoal;
end