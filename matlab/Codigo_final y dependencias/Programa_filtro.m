
clear
%%
%%POSICIONAMIENTO INICIAL DEL ROBOT
% Posición inicial
x_ini = -2; y_ini = 0; theta_ini = 0;
apoloPlaceMRobot('Pioneer3AT',[x_ini y_ini 0],theta_ini);
apoloUpdate();
apoloResetOdometry('Pioneer3AT', [x_ini y_ini theta_ini]);

%%
%%PARÁMETROS PARA EL FILTRO EKF
% Estimación inicial del estado
Xk = apoloGetOdometry('Pioneer3AT'); 

% Varianza en odometría (calibración odometría) 
var_x = 5.3272e-05;
var_y = 1.0765e-04;
var_theta = 5.7621e-06; 

% Inicialización de la matriz P
Pk = [var_x 0 0;
      0 var_y 0;
      0 0 var_theta];
  
% Algoritmo

velocidadL = 0.2; % m/s
velocidadA = 0.2; % rad/s
tiempo = 0.1; % s

%% 
%%PLANIFICACIÓN DE TRAYECTORIA
tic
imagen = imread('RawMap.pgm'); %imagen del mapa
%Declaración de la posición inicial y posición deseada del robot en
%coordenadas de Apolo

%inicio = [0,0, pi];
%fin = [-2,-3,0];

%inicio = [7.5,2.7, pi/2];
%fin = [-2,-3,0];

% inicio = [-7.5,2.7, pi/2];
inicio=[x_ini, y_ini,theta_ini];
fin = [7.5,2.7, pi/2];

%Generación de la trayectoria utilizando un algoritmo RRT
[trayectoria, arbol, mapa] = planificador(imagen, inicio, fin);
%Visualización de la trayectoria sobre el mapa
toc
show(mapa)
hold on
plot(arbol(:,1),arbol(:,2),'b.-'); %Arbol de búsqueda
plot(trayectoria(:,1),trayectoria(:,2),'r-','LineWidth',2) %Camino
plot(trayectoria(1,1), trayectoria(1,2) ,'g+', 'LineWidth',2); %Inicio
plot(trayectoria(end,1), trayectoria(end,2), 'g+', 'LineWidth',2); %Fin

%% 
%%CONTROLADOR
%Definición de los parámetros del controlador
ts = 0.1; %Tiempo de muestreo
vl =velocidadL; %0.3; %Velocidad lineal deseada
vamax = 1; %Velocidad angular máxima
va=0;
laDist = 0.3; %Distancia de seguimiento (look-ahead distance)

%Dado que este algoritmo no estabiliza al robot en el punto de destino, es
%necesario establecer una región de confianza alrededor del punto deseado
%con un radio "maxError"
maxError = 0.03; 

%Transformación de la trayectoria en coordenadas del Apolo
for i = 1:length(trayectoria)
    tray_apolo(i,:) = map2apolo(trayectoria(i,:));
    arbol_apolo(i,:) = map2apolo(arbol(i,:));
end
planTray = [tray_apolo(:,1) tray_apolo(:,2)];

%Inicialización de la posición del robot
apoloPlaceMRobot('Pioneer3AT',[inicio(1),inicio(2) 0],inicio(3));
apoloUpdate();

%Inicialización del controlador
controller = controllerInit(planTray, vl, vamax, laDist);

%Inicialización del error (distancia entre el punto final y el actual)
distanceError = norm(planTray(1,:) - planTray(end,:));
i = 1;

%Hasta que no se llegue al destino, se ejecuta el controlador
while(distanceError > maxError)
    pause(0.1)
    %Estimación de la pose con el filtro de Kalman
    Xk_1=Xk;
    Pk_1=Pk;
    [Xrealk,Xk,Pk]=funcion_EKF2(vl,va,tiempo ,Xk_1,Pk_1);
    Xreal(i,:) = Xrealk;
    Xestimado(i,:) = Xk;
    Pacumulado(1,i) = Pk(1,1);
    Pacumulado(2,i) = Pk(2,2);
    Pacumulado(3,i) = Pk(3,3);
    
    pose = Xk
    
    %Estimación de la velocidad   
    [vl,va]=control_reactivo('izquierda1','frente1','derecha1');
    if (vl==10)&&(va==10)
        [vl, va] = controller(pose);
    end

    %Corrección de la consigna de velocidad con el control reactivo
    %%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%
    
    %Movimiento del Robot
    apoloMoveMRobot('Pioneer3AT',[vl, va], ts);
    apoloUpdate();
    velocity(i,:) = [vl, va];
    error(i) = distanceError;
    %Cálculo de la distancia al punto final
    distanceError = norm([pose(1) pose(2)] - planTray(end,:));
    
    %Para evitar la situación en la cual el robot no llegue a la región de
    %destino deseada y se quede dando vueltas alrededor de dicha región, se
    %detiene el controlador cuando el error aumenta en vez de disminuir
    %si se sitúa cerca de la región de destino
%     if i>2
%         if distanceError < laDist
%             if error(i) > error(i-1)
%                 break;
%             end
%         end
%     end
    i=i+1;
end

pose3D = apoloGetLocationMRobot('Pioneer3AT')
PoseInicial = apolo2map(inicio)
PuntoFinalDeseado = apolo2map(fin)
%error = norm([PuntoFinalDeseado(1) PuntoFinalDeseado(2)] - trayectoria(end,[1 2]));
