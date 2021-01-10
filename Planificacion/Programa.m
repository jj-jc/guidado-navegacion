
clear
%% 
%%PLANIFICACIÓN DE TRAYECTORIA
imagen = imread('RawMap.pgm'); %imagen del mapa
%Declaración de la posición inicial y posición deseada del robot en
%coordenadas de Apolo
inicio = [0,0, pi];
fin = [-2,-3,0];

%Generación de la trayectoria utilizando un algoritmo RRT
[trayectoria, arbol, mapa] = planificador(imagen, inicio, fin);
%Visualización de la trayectoria sobre el mapa
show(mapa)
hold on
plot(arbol(:,1),arbol(:,2),'b.-'); %Arbol de búsqueda
plot(trayectoria(:,1),trayectoria(:,2),'r-','LineWidth',2) %Camino
%% 
%%CONTROLADOR
%Definición de los parámetros del controlador
ts = 0.1; %Tiempo de muestreo
vl = 0.3; %Velocidad lineal deseada
vamax = 2; %Velocidad angular máxima
laDist = 0.3; %Distancia de seguimiento (look-ahead distance)

%Dado que este algoritmo no estabiliza al robot en el punto de destino, es
%necesario establecer una región de confianza alrededor del punto deseado
%con un radio "maxError"
maxError = 0.03; 

%Transformación de la trayectoria en coordenadas del
for i = 1:length(trayectoria)
    tray_apolo(i,:) = map2apolo(trayectoria(i,:));
    arbol_apolo(i,:) = map2apolo(arbol(i,:));
end
controlTray = [tray_apolo(:,1) tray_apolo(:,2)];

%Inicialización de la posición del robot
apoloPlaceMRobot('Pioneer3AT',[inicio(1),inicio(2) 0],inicio(3));
apoloUpdate();

%Llamada al controlador
controller = controllerInit(controlTray, vl, vamax, laDist);

%Inicialización del error (distancia entre el punto final y el actual)
distanceError = norm(controlTray(1,:) - controlTray(end,:));
i = 1;
while(distanceError > maxError)
    %La funcion control calcula la acción de control, la ejecuta y devuelve
    %el nuevo error de distancia después del movimiento
    distanceError = control(ts, controlTray, controller);
    error(i) = distanceError;
    pose3D = apoloGetLocationMRobot('Pioneer3AT');
    posx(i) = pose3D(1);
    posy(i) = pose3D(2);
    
    %Para evitar la situación en la cual el robot no llegue a la región de
    %destino deseada y se quede dando vueltas alrededor de dicha región, se
    %detiene el controlador cuando el error aumenta en vez de disminuir
    %cuando se sitúa cerca de la región de destino
    if i>2
        if distanceError < laDist
            if error(i) > error(i-1)
                break;
            end
        end
    end
    i=i+1;
end

figure;
plot(posy,-posx);
figure;
plot(error);
