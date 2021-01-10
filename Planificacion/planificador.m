function [trayectoria, arbol, map] = planificador(mapa, inicio, fin)

%%GENERACIÓN DEL MAPA A PARTIR DE UNA IMAGEN
%Transformación a las coordenadas del mapa de ocupación binario que se
%generará 
inicio_mapa = apolo2map(inicio);
fin_mapa = apolo2map(fin);

%Se debe normalizar la imagen e invertir para la generación del mapa de 
%ocupación
mapaNorm = double(mapa)/255; 
mapaOccupancy = 1 - mapaNorm; 
%Obtención del mapa de ocupación
map = binaryOccupancyMap(mapaOccupancy,16.375);

%Para tener en cuenta el ancho del robot y evitar colisiones,
%se engordan los obstáculos en función de dicho ancho
robotwidth = 0.35;
inflate(map, robotwidth/2); 

%First, a a state space with states[x, y, theta] is generated
ss = stateSpaceSE2; 

%Creates validator that checks the occupancy map for collision detection
sv = validatorOccupancyMap(ss); 

%Loads the map into the validator
sv.Map = map; 

%Sets the interval of distance for collision detection. 
%This determins the accuracy of the collision detection.
sv.ValidationDistance = 0.001; 

%Sets the room limits
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]]; 

%Creates an RRT planner using the state space and the validator
planner = plannerRRT(ss,sv);   

%Sets the maximum distance between the nodes of the tree
planner.MaxConnectionDistance = 0.2;
planner.GoalReachedFcn = @maxErrorPlanner;
%Generates the path using an RRT algorithm between the start and the goal
[path,tree] = plan(planner,inicio_mapa,fin_mapa);
arbol= tree.TreeData;
trayectoria = path.States;
end
