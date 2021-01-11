%Pruebas controlador para la memoria
clear
ts = 0.1; %Tiempo de muestreo
vl = 0.3; %Velocidad lineal deseada
vamax = 2; %Velocidad angular máxima
laDist = 0.15; %Distancia de seguimiento (look-ahead distance)
maxError = 0.03; 

%Trayectoria de pruebas
r=1;
th = 0:pi/40:4*pi;
xunit = r * cos(th) - 2;
yunit = -3+(1:1:length(th))/28;
trayectoria_pruebas = transpose([xunit ; yunit]);
%plot(xunit, yunit);

controller = controllerInit(trayectoria_pruebas, vl, vamax, laDist);
apoloPlaceMRobot('Pioneer3AT',[xunit(1),yunit(1) 0],pi/2);
apoloUpdate();
distanceError = norm(trayectoria_pruebas(1,:) - trayectoria_pruebas(end,:));
i = 1;

while(distanceError > maxError)
    
    %Estimación de la pose con el filtro de Kalman
    pose3D = apoloGetLocationMRobot('Pioneer3AT');
    pose = [pose3D(1) pose3D(2) pose3D(4)];
    poseRobot(:,i) = pose;
    
    %Estimación de la velocidad con el controlador
    [vl, va] = controller(pose);

    %Movimiento del Robot
    apoloMoveMRobot('Pioneer3AT',[vl, va], ts);
    apoloUpdate();
    velocity(i,:) = [vl, va];
    error(i) = distanceError;
    %Cálculo de la distancia al punto final
    distanceError = norm([pose(1) pose(2)] - trayectoria_pruebas(end,:));
    
    %Para evitar la situación en la cual el robot no llegue a la región de
    %destino deseada y se quede dando vueltas alrededor de dicha región, se
    %detiene el controlador cuando el error aumenta en vez de disminuir
    %si se sitúa cerca de la región de destino
    if i>2
        if distanceError < laDist
            if error(i) > error(i-1)
                break;
            end
        end
    end
    i=i+1;
end
plot(poseRobot(2,:), poseRobot(1,:), 'r', 'LineWidth',1.5);
hold on;
plot(trayectoria_pruebas(:,2), trayectoria_pruebas(:,1), 'b', 'LineWidth',1.5);
legend("Trayectoria del robot","Trayectoria deseada");
ylim([-3.5 -0.5]);
xlim([-2.8571 2.8571]);
xlabel("X (m)");
ylabel("Y (m)");
set(gca,'fontsize', 12)
annotation('textbox', [0.15, 0.8, 0.25, 0.1], 'String', "look-ahead = 0.15 m avance = 3 m/s")