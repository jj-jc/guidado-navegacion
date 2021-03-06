
clear
%%
%%POSICIONAMIENTO INICIAL DEL ROBOT
% Posición inicial

%Puntos Simulacion 1
x_ini = 6.5; y_ini = -2.7; theta_ini = pi;
x_fin = 6.5; y_fin = 2.7; theta_fin = pi/2;

%Puntos Simulacion 2
%x_ini = 6.5; y_ini = 2.7; theta_ini = pi;
%x_fin = -6.5; y_fin = -6.5; theta_fin = pi/2;

%Puntos Simulacion 3
%x_ini = -6.5; y_ini = 6.5; theta_ini = 0;
%x_fin = 6.5; y_fin = -2.7; theta_fin = 0;

%Puntos Simulacion 4
%x_ini = -6.5; y_ini = -6.5; theta_ini = 0;
%x_fin = -6.5; y_fin = 6.5; theta_fin = pi;

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

velocidadL = 0.1; % m/s
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
inicio=[x_ini, y_ini, theta_ini];
fin = [x_fin,y_fin, theta_fin];

%Generación de la trayectoria utilizando un algoritmo RRT
[trayectoria, arbol, mapa] = planificador(imagen, inicio, fin);
%Visualización de la trayectoria sobre el mapa
toc
figure;
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
maxError = 0.05; 

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
figure;
pause(10);
while(distanceError > maxError)
    pause(0.02)
    %Estimación de la pose con el filtro de Kalman
    Xk_1=Xk;
    Pk_1=Pk;
    [Xrealk,Xk,Pk]=funcion_EKF2(Xk_1,Pk_1);
    Xreal(i,:) = Xrealk;
    Xestimado(i,:) = Xk;
    Pacumulado(1,i) = Pk(1,1);
    Pacumulado(2,i) = Pk(2,2);
    Pacumulado(3,i) = Pk(3,3);
    

    pose = Xk;
    
    %Estimación de la velocidad   
    [vl,va]=control_reactivo('izquierda1','frente1','derecha1');
    if (vl==10)&&(va==10)
        [vl, va] = controller(pose);
        controllerAction(i) = 0;
    else 
        controllerAction(i) = 1;
    end
    plot(planTray(:,2),-planTray(:,1),'b', 'LineWidth', 1.5);
    hold on;
    plot(Xestimado(:,2), -Xestimado(:,1),'r', 'LineWidth' , 1.5);
    hold on;
    plot(Xreal(:,2), -Xreal(:,1),'g', 'LineWidth', 1.5);
    hold on;
    %plot(controllerAction);
    %Movimiento del Robot
    apoloMoveMRobot('Pioneer3AT',[vl, va], ts);
    apoloUpdate();
    velocity(i,:) = [vl, va];
    error(i) = distanceError;
    %Cálculo de la distancia al punto final
    distanceError = norm([pose(1) pose(2)] - planTray(end,:));
    
    i=i+1;
end
legend("Trayectoria planificada","Trayectoria estimada","Trayectoria real");
xlabel("Y (m)");
ylabel("X (m)");
set(gca,'fontsize', 12);
ylim([0.5, 7.5]);
