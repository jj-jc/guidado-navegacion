
%Map generation
image = imread('RawMap.pgm'); %262x262 bits image
imageNorm = double(image)/255; %Normalized image
imageOccupancy = 1 - imageNorm; %Invert the image for occupancy map
map = binaryOccupancyMap(imageOccupancy,16.375); %Gets the grid map
robotwidth = 0.35; %Robot width
inflate(map, robotwidth/2); %Inflate obstacles to avoid robot collision

ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.001;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
planner = plannerRRT(ss,sv);
planner.MaxConnectionDistance = 0.6;
start = [0.5,0.5,0];
goal = [8,15.6,0];
%rng(100,'twister'); % for repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);
show(map)
hold on
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2) % draw path

