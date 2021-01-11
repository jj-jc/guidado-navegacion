<<<<<<< HEAD
=======
clc, clear
>>>>>>> ac53b2de440325c3ad877c3c75796e87a816263f
% Sistema de locomoción tipo diferencial

% Determinación de la incertidumbre del modelo cinemático

<<<<<<< HEAD
% 1) Se coloca el robot en el origen de coordenadas del mundo
apoloPlaceMRobot('Pioneer3AT',[0 0 0],0);
apoloUpdate();
posicion_real = apoloGetLocationMRobot('Pioneer3AT'); % Se calcula la pose actual
pos_real =[posicion_real(1),posicion_real(2),posicion_real(4)]; 
=======
% 1) Se obtiene la posición actual del robot
apoloPlaceMRobot('Pioneer3AT',[0.5 0.5 0],0.5);
apoloUpdate();
posicion_real = apoloGetLocationMRobot('Pioneer3AT')
pos_real =[posicion_real(1),posicion_real(2),posicion_real(4)] % al parecer 'theta' y 'rz' tienen sentidos opuestos
>>>>>>> ac53b2de440325c3ad877c3c75796e87a816263f

% 2) Se establece como offset inicial de la odometría la posición actual
apoloResetOdometry('Pioneer3AT');
posicion_odometria = apoloGetOdometry('Pioneer3AT',pos_real);

% 3) Se mueve el robot y se mide con la odometria
for i = 1:3000
    apoloMoveMRobot('Pioneer3AT',[0.5, 0.5], 0.1);
    apoloUpdate();
    % Se almacenan en 'posicion_odometria' todos los datos de la odometría
    pos_odometria = apoloGetOdometry('Pioneer3AT');
    posicion_odometria = [posicion_odometria; pos_odometria]; 
    % Se almacenan en 'pos_real' todos los datos de la pose real del robot
    pos_actual = apoloGetLocationMRobot('Pioneer3AT');
    pos_real = [pos_real; pos_actual(1),pos_actual(2),pos_actual(4)];
end

% 4) Se dibuja el recorrido real y el de la odometria
plot(posicion_odometria(:,1),posicion_odometria(:,2),'b')
xlabel('X (m)')
ylabel('Y (m)')
hold on;
plot(pos_real(:,1),pos_real(:,2),'r')
legend('Pose odometria (azul)','Pose real(rojo)')

% 5) Se calculan las medias y varianzas de cada error de las variables de la odometría
error = abs(posicion_odometria - pos_real); % Matriz de errores absolutos entre odometría y posición real
error(:,3) = mod(error(:,3),pi);
for i = 1:length(error)
    if(error(i,3) > 3)
        error(i,3) = error(i,3)-pi;
    end
end

media = mean(error); % Se obtiene la media del error para cada columna 
desviacion_estandar = std(error); % Se obtiene la desviacion estandar del error para cada columna
var_x = (desviacion_estandar(1))^2;
var_y = (desviacion_estandar(2))^2;
var_theta = (desviacion_estandar(3))^2;

% Se representa el error en x y en y
figure;
plot(error(:,1),'r')
xlabel('Muestras')
ylabel('Error (m)')
hold on;
plot(error(:,2),'b')
legend('Error en x','Error en y')

% Se representa el error en ángulo
figure
plot(error(:,3))
xlabel('Muestras')
ylabel('Error (rad)')

% Se imprime por consola los datos de las medias y varianzas
disp('---------------------------------------------------------------------------')
disp('----------------INCERTIDUMBRE DEL MODELO CINEMÁTICO------------------------')
disp(' ')
disp('Valor medio de los errores absolutos cometidos con la odometría:')
disp(['Error absoluto medio en coordenada x-------------> ', num2str(media(1)),' metros'])
disp(['Error absoluto medio en coordenada y-------------> ', num2str(media(2)),' metros'])
disp(['Error absoluto medio en la orientacion theta-----> ', num2str(media(3)),' rad'])
disp(' ')
disp('Varianza correspondiente a cada variable de la odometría:')
disp(['Varianza de la coordenada x----------> ', num2str(var_x),' metros'])
disp(['Varianza de la coordenada y----------> ', num2str(var_y),' metros'])
disp(['Varianza de la orientacion theta-----> ', num2str(var_theta),' rad'])
disp(' ')
disp('---------------------------------------------------------------------------')
