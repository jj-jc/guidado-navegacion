% Sistema de locomoción tipo diferencial

% Determinación de la incertidumbre del modelo cinemático

% 1) Se coloca el robot en el origen de coordenadas del mundo
apoloPlaceMRobot('Pioneer3AT',[0 0 0],0);
apoloUpdate();
posicion_real = apoloGetLocationMRobot('Pioneer3AT'); 
pos_real =[posicion_real(1),posicion_real(2),posicion_real(4)]; 

% 2) Se establece como offset inicial de la odometría la posición actual
apoloResetOdometry('Pioneer3AT');
posicion_odometria = apoloGetOdometry('Pioneer3AT',pos_real);