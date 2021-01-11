
% apoloResetOdometry('Pioneer3AT');
% apoloPlaceMRobot('Pioneer3AT',[-2 -2 0],0);
% apoloUpdate();
% pause(1);
% posicion_real = apoloGetLocationMRobot('Pioneer3AT')
% apoloResetOdometry('Pioneer3AT',[-2 -2 0]);
% pos_odometry = apoloGetOdometry('Pioneer3AT')
% apoloMoveMRobot('Pioneer3AT',[0.5, 0], 6); % Avanza 3 metros en x
% apoloUpdate();
% pause(1);
% posicion_real = apoloGetLocationMRobot('Pioneer3AT')
% pos_odometry = apoloGetOdometry('Pioneer3AT')
% apoloMoveMRobot('Pioneer3AT',[0, 0.5],-pi); % Rota -90 grados
% apoloUpdate();
% pause(1);
% apoloMoveMRobot('Pioneer3AT',[0.5, 0],6); % Avanza 3 en -y
% apoloUpdate();
% pause(1);
% posicion_real = apoloGetLocationMRobot('Pioneer3AT')
% pos_odometry = apoloGetOdometry('Pioneer3AT') 

apoloResetOdometry('Pioneer3AT');
apoloPlaceMRobot('Pioneer3AT',[-2 -0.5 0],-pi/6);
apoloUpdate();
pause(1);
posicion_real = apoloGetLocationMRobot('Pioneer3AT')
apoloResetOdometry('Pioneer3AT',[-2 -0.5 -pi/6]);
pos_odometry = apoloGetOdometry('Pioneer3AT')
apoloMoveMRobot('Pioneer3AT',[1, 0], 0.5); % Avanza 0.5 metros en -y
apoloUpdate();
pause(1);
posicion_real = apoloGetLocationMRobot('Pioneer3AT')
pos_odometry = apoloGetOdometry('Pioneer3AT')
apoloMoveMRobot('Pioneer3AT',[0, 0.5],pi); % Rota 90 grados
apoloUpdate();
pause(1);
posicion_real = apoloGetLocationMRobot('Pioneer3AT')
pos_odometry = apoloGetOdometry('Pioneer3AT')
apoloMoveMRobot('Pioneer3AT',[0.5, 0],6); % Avanza 3 en x
apoloUpdate();
pause(1);
posicion_real = apoloGetLocationMRobot('Pioneer3AT')
pos_odometry = apoloGetOdometry('Pioneer3AT')

% apoloResetOdometry('Pioneer3AT');
% apoloPlaceMRobot('Pioneer3AT',[0 0 0],0);
% apoloUpdate();
% pause(1);
% posicion_real = apoloGetLocationMRobot('Pioneer3AT')
% apoloResetOdometry('Pioneer3AT',[0 0 0]);
% posicion_odometry = apoloGetOdometry('Pioneer3AT')
% 
% apoloMoveMRobot('Pioneer3AT',[0, 0.5],2*pi); % Rota 180 grados
% apoloUpdate();
% pause(1);
% posicion_real = [posicion_real; apoloGetLocationMRobot('Pioneer3AT')];
% posicion_odometry = [posicion_odometry; apoloGetOdometry('Pioneer3AT')];
% 
% apoloMoveMRobot('Pioneer3AT',[0.5, 0], 4); % Avanza 2 metros
% apoloUpdate();
% pause(1);
% posicion_real = [posicion_real; apoloGetLocationMRobot('Pioneer3AT')];
% posicion_odometry = [posicion_odometry; apoloGetOdometry('Pioneer3AT')];
% 
% apoloMoveMRobot('Pioneer3AT',[0, 0.5],pi); % Rota 90 grados
% apoloUpdate();
% pause(1);
% posicion_real = [posicion_real; apoloGetLocationMRobot('Pioneer3AT')];
% posicion_odometry = [posicion_odometry; apoloGetOdometry('Pioneer3AT')];
% 
% 
% apoloMoveMRobot('Pioneer3AT',[0.5, 0], 10); % Avanza 3 metros en -y
% apoloUpdate();
% pause(1);
% posicion_real = [posicion_real; apoloGetLocationMRobot('Pioneer3AT')];
% posicion_odometry = [posicion_odometry; apoloGetOdometry('Pioneer3AT')];
% 
% apoloMoveMRobot('Pioneer3AT',[0, 0.5],pi); % Rota 90 grados
% apoloUpdate();
% pause(1);
% posicion_real = [posicion_real; apoloGetLocationMRobot('Pioneer3AT')];
% posicion_odometry = [posicion_odometry; apoloGetOdometry('Pioneer3AT')];
% 
% apoloMoveMRobot('Pioneer3AT',[0.5, 0],6); % Avanza 3 en x
% apoloUpdate();
% pause(1);
% posicion_real = [posicion_real; apoloGetLocationMRobot('Pioneer3AT')];
% posicion_odometry = [posicion_odometry; apoloGetOdometry('Pioneer3AT')];
% 
% 
