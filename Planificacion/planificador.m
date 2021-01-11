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
robotwidth = 0.497;
inflate(map, robotwidth/2); 

%Primero se genera un espacio de estados[x, y, theta]
ss = stateSpaceSE2; 

%Se crea un validador que compruebas las colisiones del robot en el mapa
sv = validatorOccupancyMap(ss); 

%Carga el mapa en el validador
sv.Map = map; 

%Se especifica los intervalos de distancia utilizados para la detección de
%las colisiones. Cuanto menor sea, más tardará en ejecutarse el algoritmo
%pero mayor precisión tendrá
sv.ValidationDistance = 0.001; 

%Se establecen los límites del entorno
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]]; 

%Se crea un planificador utilizando el algoritmo RRT al que se pasa el
%validador y el espacio de estados
planner = plannerRRT(ss,sv);   

%Se establece la distancia máxima entre dos nodos del árbol
planner.MaxConnectionDistance = 0.2;
planner.GoalReachedFcn = @maxErrorPlanner;

%Se genera la trayectoria utilizando el algoritmo RRT
[path,tree] = plan(planner,inicio_mapa,fin_mapa);
arbol= tree.TreeData;
trayectoria = path.States;
end
