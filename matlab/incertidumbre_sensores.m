
% Sistema de percepción

% Modelado de la incertidumbre de los sensores exteroceptivos
% mediante la calibración de los mismos

% Sensores utilizados: 3 ultrasonidos y 1 telémetro láser

% Se coloca el robot mirando hacia la mesa principal
apoloPlaceMRobot('Pioneer3AT',[-2 0 0],pi); 
apoloUpdate();

% 1) Calibración de los ultrasonidos

% Se mide varias veces con los ultrasonidos
dist_ultrasonidos = apoloGetAllultrasonicSensors('Pioneer3AT');
for i = 1:3000
    dist_ultrasonidos = [dist_ultrasonidos; apoloGetAllultrasonicSensors('Pioneer3AT')];
    apoloUpdate();
end

% Se calcula la media de las medidas de cada ultrasonido
media = mean(dist_ultrasonidos);

% Se calcula la varianza correspondiente a la medida de cada ultrasonidos
desviacion_std = std(dist_ultrasonidos);
var_us_frente = (desviacion_std(1))^2;
var_us_izq = (desviacion_std(2))^2;
var_us_dcha = (desviacion_std(3))^2;

% Se imprime por consola los datos de las medias y varianzas
disp('-----------------------------------------------------------------------')
disp('----------------CALIBRACION DE LOS ULTRASONIDOS------------------------')
disp(' ')
disp('Valor medio de las distancias medidas por los ultrasonidos:')
disp(['Distancia media del ultrasonidos frontal-----------> ', num2str(media(1)),' metros'])
disp(['Distancia media del ultrasonidos de la izquierda---> ', num2str(media(2)),' metros'])
disp(['Distancia media del ultrasonidos de la derecha-----> ', num2str(media(3)),' metros'])
disp(' ')
disp('Varianza correspondiente a las medidas de cada ultrasonidos:')
disp(['Varianza del ultrasonidos frontal-----------> ', num2str(var_us_frente),' metros'])
disp(['Varianza del ultrasonidos de la izquierda---> ', num2str(var_us_izq),' metros'])
disp(['Varianza del ultrasonidos de la derecha-----> ', num2str(var_us_dcha),' metros'])
disp(' ')

% 2) Calibración del telémetro láser

% Se mide con el telémetro láser 2 balizas colocadas en el entorno
medida_laser = apoloGetLaserLandMarks('LMS100');
distancias = medida_laser.distance;
angulos = medida_laser.angle;
for i = 1:3000
    medida_laser = apoloGetLaserLandMarks('LMS100');
    distancias = [distancias; medida_laser.distance];
    angulos = [angulos; medida_laser.angle];
    apoloUpdate()
end

% Se calcula la media de las distancias y los ángulos a cada baliza
media_dist = mean(distancias);
media_ang = mean(angulos);

% Se calcula la varianza de las distancias y los angulos a cada baliza

% Distancias
desviacion_std_dist = std(distancias);
var_dist_LM1 = (desviacion_std_dist(1))^2;
var_dist_LM2 = (desviacion_std_dist(2))^2;

% Ángulos
desviacion_std_ang = std(angulos);
var_ang_LM1 = (desviacion_std_ang(1))^2;
var_ang_LM2 = (desviacion_std_ang(2))^2;

% Se imprime por consola los datos de las medias y varianzas
disp('----------------CALIBRACION DEL TELEMETRO LASER------------------------')
disp(' ')
disp('Valor medio de las distancias medidas por el laser a cada baliza:')
disp(['Distancia media a la baliza LM1-----> ', num2str(media_dist(1)),' metros'])
disp(['Distancia media a la baliza LM2-----> ', num2str(media_dist(2)),' metros'])
disp(' ')
disp('Valor medio del ángulo al que se sitúan las balizas respecto el laser:')
disp(['Angulo medio respecto a la baliza LM1----> ', num2str(media_ang(1)),' rad'])
disp(['Ángulo medio respecto a la baliza LM2----> ', num2str(media_ang(2)),' rad'])
disp(' ')
disp('Varianza correspondiente a las medidas de distancias hechas por el laser:')
disp(['Varianza de la medida de distancia a la baliza LM1-----> ', num2str(var_dist_LM1),' metros'])
disp(['Varianza de la medida de distancia a la baliza LM2-----> ', num2str(var_dist_LM2),' metros'])
disp(' ')
disp('Varianza correspondiente a las medidas de ángulos hechas por el laser:')
disp(['Varianza de la medida de ángulo a la baliza LM1-----> ', num2str(var_ang_LM1),' rad'])
disp(['Varianza de la medida de ángulo a la baliza LM2-----> ', num2str(var_ang_LM2),' rad'])
disp(' ')
disp('-----------------------------------------------------------------------')

