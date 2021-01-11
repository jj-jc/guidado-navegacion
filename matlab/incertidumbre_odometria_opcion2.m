% Sistema de locomoción tipo diferencial

% Determinación de la incertidumbre del modelo cinemático

% Se coloca el robot en una posicion inicial y se reseta odometría
apoloPlaceMRobot('Pioneer3AT',[-2 0 0],0);
apoloUpdate();
apoloResetOdometry('Pioneer3AT',[-2 0 0]);

% Se mueve el robot y se mide la posicion real y con la odometria
for i = 1:50
        apoloMoveMRobot('Pioneer3AT',[0.1, 0.1], 0.1);
        apoloUpdate();
end
posicion_odometria = apoloGetOdometry('Pioneer3AT'); % posicion del robot dada por la odemetria despues de moverse
posicion_real = apoloGetLocationMRobot('Pioneer3AT'); % posicion real del robot despues de moverse
pos_real =[posicion_real(1),posicion_real(2),posicion_real(4)]; 

% Se repite el mismo movimiento y se va almacenado la posicion de destino
% dada por la odometria en una matriz
for j = 1:500
    % Se vuelve a la posicion inicial y se reseta la odometría
    apoloPlaceMRobot('Pioneer3AT',[-2 0 0],0);
    apoloUpdate();
    apoloResetOdometry('Pioneer3AT',[-2 0 0]);
    % Se mueve el robot
    for i = 1:50
        apoloMoveMRobot('Pioneer3AT',[0.1, 0.1], 0.1);
        apoloUpdate();
    end
    % Se almacenan en una matriz todas las medidas de la odometría
    pos_odometria = apoloGetOdometry('Pioneer3AT');
    posicion_odometria = [posicion_odometria; pos_odometria]; 
%     % Se almacenan en 'pos_real' todos los datos de la posicion real del robot
%     pos_actual = apoloGetLocationMRobot('Pioneer3AT');
%     pos_real = [pos_real; pos_actual(1),pos_actual(2),pos_actual(4)];
end

% Se calcula el valor medio de la posicion final medida con la odometría
media = (posicion_odometria);

% Se calcula el error absoluto entre el valor medio y el valor real
error_x = abs(media(1)- posicion_real(1));
error_y = abs(media(2)- posicion_real(2));
error_theta = abs(media(3)- posicion_real(4));

% Se calcula la varianza de esas medidas
desviacion_estandar = posicion_odometria;
var_x = (desviacion_estandar(1))^2;
var_y = (desviacion_estandar(2))^2;
var_theta = (desviacion_estandar(3))^2;

% Se imprime por consola los datos de las medias y varianzas
disp('---------------------------------------------------------------------------')
disp('----------------INCERTIDUMBRE DEL MODELO CINEMÁTICO------------------------')
disp(' ')
disp('Valor medio de las coordenadas de la posicion medidos con la odometría:')
disp(['Valor medio coordenada x-------------> ', num2str(media(1)),' metros'])
disp(['Valor medio coordenada y-------------> ', num2str(media(2)),' metros'])
disp(['Valor medio orientacion theta--------> ', num2str(media(3)),' rad'])
disp(' ')
disp('Valor real de las coordenadas de la posicion:')
disp(['Coordenada x-------------> ', num2str(posicion_real(1)),' metros'])
disp(['Coordenada y-------------> ', num2str(posicion_real(2)),' metros'])
disp(['Orientacion theta--------> ', num2str(posicion_real(4)),' rad'])
disp(' ')
disp('Varianza correspondiente a cada variable de la odometría:')
disp(['Varianza de la coordenada x----------> ', num2str(var_x),' metros'])
disp(['Varianza de la coordenada y----------> ', num2str(var_y),' metros'])
disp(['Varianza de la orientacion theta-----> ', num2str(var_theta),' rad'])
disp(' ')
disp('---------------------------------------------------------------------------')