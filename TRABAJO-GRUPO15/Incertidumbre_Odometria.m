
% Sistema de locomoción tipo diferencial

% Determinación de la incertidumbre del modelo cinemático

% 1) Se calcula la posición y orientación del robot
apoloPlaceMRobot('Pioneer3AT',[0 0 0],0);
apoloUpdate();
posicion_real = apoloGetLocationMRobot('Pioneer3AT');
pos_real =[posicion_real(1),posicion_real(2),posicion_real(4)];

% 2) Se resetea la odometría
apoloResetOdometry('Pioneer3AT');
posicion_odometria = apoloGetOdometry('Pioneer3AT');

% 3) Se establece la velocidad lineal y angular del movimiento
velocidadL = 0.5; % m/s
velocidadA = 0.3; % rad/s
tiempo = 0.1; % s

% 4) Se declaran las matrices donde se almacena el giro y el avance en cada
% iteración
n = 1000;
avance_odometria = zeros(n,1);
giro_odometria = zeros(n,1);
avance_real = zeros(n,1);
giro_real = zeros(n,1);

% 5) Se mueve el robot y se mide con la odometria
for i = 1:n
    apoloMoveMRobot('Pioneer3AT',[velocidadL, velocidadA], tiempo);
    apoloUpdate();
    % Se almacenan todos los datos de la odometría
    pos_odometria = apoloGetOdometry('Pioneer3AT');
    posicion_odometria = [posicion_odometria; pos_odometria];
    % Se calcula el avance y giro según odometría
    avance_odometria(i,1) = sqrt((posicion_odometria(i+1,1)-posicion_odometria(i,1))^2 + (posicion_odometria(i+1,2)-posicion_odometria(i,2))^2);
    giro_odometria(i,1) = abs(posicion_odometria(i+1,3))-abs(posicion_odometria(i,3));
    % Se almacenan todos los datos de la pose real del robot
    pos_actual = apoloGetLocationMRobot('Pioneer3AT');
    pos_real = [pos_real; pos_actual(1),pos_actual(2),pos_actual(4)];
    % Se calcula el avance y giro real
    avance_real(i,1) = sqrt((pos_real(i+1,1)-pos_real(i,1))^2 + (pos_real(i+1,2)-pos_real(i,2))^2);
    giro_real(i,1) = abs(pos_real(i+1,3))-abs(pos_real(i,3));
end

% 6) Se dibuja el recorrido real y el de la odometria
plot(posicion_odometria(:,1),posicion_odometria(:,2),'b')
xlabel('X (m)')
ylabel('Y (m)')
hold on;
plot(pos_real(:,1),pos_real(:,2),'r')
legend('Posicion odometria (azul)','Posicion real (rojo)')

% 7) Se calculan las medias y varianzas del error de cada variable de la odometría
error_pos = abs(posicion_odometria - pos_real); % Matriz de errores absolutos entre odometría y posición real
error_pos(:,3) = mod(error_pos(:,3),pi);
for i = 1:length(error_pos)
    if(error_pos(i,3) > 3)
        error_pos(i,3) = abs(error_pos(i,3)-pi);
    end
end

media = mean(error_pos); % Se obtiene la media del error para cada columna 
desviacion_estandar = std(error_pos); % Se obtiene la desviacion estandar del error para cada columna
var_x = (desviacion_estandar(1))^2;
var_y = (desviacion_estandar(2))^2;
var_theta = (desviacion_estandar(3))^2;

% Se representa el error de posicion en x y en y
figure;
plot(error_pos(:,1),'r')
xlabel('Muestras')
ylabel('Error (m)')
hold on;
plot(error_pos(:,2),'b')
legend('Error en x (rojo)','Error en y (azul)')

% Se representa el error en ángulo
figure;
plot(error_pos(:,3))
xlabel('Muestras')
ylabel('Error (rad)')

% 8) Se calcula la media y la varianza del error en avance y giro
error_avance = abs(avance_odometria-avance_real);
media_err_avance = mean(error_avance);
desv_std_avance = std(error_avance);
var_avance = (desv_std_avance)^2;

error_giro = abs(giro_odometria-giro_real);
for i= 1:length(error_giro)
    if error_giro(i,1) > 0.01
        error_giro(i,1) = NaN;
    end
end
media_err_giro = mean(error_giro, 'omitnan');
desv_std_giro = std(error_giro, 'omitnan');
var_giro= (desv_std_giro)^2;

% Se representa el error del avance
figure;
plot(error_avance(:,1),'r')
xlabel('Muestras')
ylabel('Error en avance (m)')

% Se representa el error del giro
figure;
plot(error_giro(:,1))
xlabel('Muestras')
ylabel('Error del giro (rad)')

% Se imprime por consola los datos de las medias y varianzas
disp('---------------------------------------------------------------------------')
disp('----------------INCERTIDUMBRE DEL MODELO CINEMÁTICO------------------------')
disp(' ')
disp('Valor medio de los errores absolutos cometidos con la odometría:')
disp(['Error absoluto medio en coordenada x-------------> ', num2str(media(1)),' metros'])
disp(['Error absoluto medio en coordenada y-------------> ', num2str(media(2)),' metros'])
disp(['Error absoluto medio en la orientacion theta-----> ', num2str(media(3)),' rad'])
disp(['Error absoluto medio en avance ------------------> ', num2str(media_err_avance),' metros'])
disp(['Error absoluto medio en el giro -----------------> ', num2str(media_err_giro),' rad'])
disp(' ')
disp('Varianza correspondiente a cada variable de la odometría:')
disp(['Varianza de la coordenada x----------> ', num2str(var_x)])
disp(['Varianza de la coordenada y----------> ', num2str(var_y)])
disp(['Varianza de la orientacion theta-----> ', num2str(var_theta)])
disp(['Varianza del avance------------------> ', num2str(var_avance)])
disp(['Varianza del giro -------------------> ', num2str(var_giro)])
disp(' ')
disp('---------------------------------------------------------------------------')
