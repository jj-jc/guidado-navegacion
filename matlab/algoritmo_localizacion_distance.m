% Algoritmo de localización (estimación del estado)
% Filtro de Kalman


% Definimos una trayectoria circular
velocidadL = 0.2;  % Velocidad lineal 0.2 m/seg
timestep = 0.5;  % Actualizacion de sensores
distanciatotal = 4*pi; % en metros.. radio de 2 metros
numerodepasos = ceil(distanciatotal/(velocidadL*timestep));
velocidadA = 2*pi/(numerodepasos*timestep); % Velocidad angular

% Varianza del ruido del proceso 
Qd = 0.01*velocidadL*timestep;
Qb = 0.02*velocidadA*timestep;
Qk_1 = [Qd 0; 0 Qb];

for k = 1:2*numerodepasos
    trayectoriaD(k) = velocidadL*timestep;
    trayectoriaB(k) = velocidadA*timestep;
    trayectoriaDRuido(k) = velocidadL*timestep + sqrt(Qd)*randn;
    trayectoriaBRuido(k) = velocidadA*timestep + sqrt(Qb)*randn;
end


    % Para acortar el nombre de la variable
    Uk = [trayectoriaDRuido(1); trayectoriaBRuido(1)];

% Inicialización:
    
    % Posición inicial
    x_ini = -2;
    y_ini = 0;
    theta_ini = 0;
    Xrealk = [x_ini; y_ini; theta_ini];
    apoloPlaceMRobot('Pioneer3AT',[x_ini y_ini 0],theta_ini);
    apoloUpdate();

    % Estimación inicial del estado
    Xk = apoloGetOdometry('Pioneer3AT'); 

    % Varianza en odometría (calibración odometría) 
    var_x = 2.669e-05;
    var_y = 3.085e-05;
    var_theta = 3.0046e-06;
    
    % Inicialización de la matriz P
    Pk = [var_x 0 0;
          0 var_y 0;
          0 0 var_theta];

    % Varianza en la medida (calibración del laser)
    var_dist = 0.00028562;
    var_ang = 0.0001961;
    
    % Inicialización de matriz R 
    Rk = [var_ang 0 0;
          0 var_ang 0;
          0 0 var_ang];

    % Posición de las balizas:
    LM_xy = [-3.9 -3.0;    % LM1
               4.0  7.9;   % LM2
               4.0 -7.9;   % LM3
               4.0  0.1;   % LM4
               4.0 -0.1;   % LM5
              -3.9  7.9;   % LM6
              -3.9 -7.9;   % LM7
              -3.9  3.0;   % LM8
               7.9 -0.1;   % LM9
               7.9  0.1];  % LM10
              
    
% Algoritmo de localización

    % Detección de balizas
    laser = apoloGetLaserLandMarks('LMS100');
    num_balizas = length(laser.id); % numero de balizas detectadas
    if num_balizas > 2 % Aplicar filtro cuando se detecten 3 balizas mínimo
        % Nuevo ciclo: k-1 = k
            Xk_1 = Xk;
            Pk_1 = Pk;
            
        % Etapa 1: Predicción
            % Del estado
            X_k = apoloGetOdometry('Pioneer3AT');
            
              Ak = [1 0 (-Uk(1)*sin(Xk_1(3)+Uk(2)/2));
              0 1 (Uk(1)*cos(Xk_1(3)+Uk(2)/2));
              0 0 1                             ];
              Bk = [(cos(Xk_1(3)+Uk(2)/2)) (-0.5*Uk(1)*sin(Xk_1(3)+Uk(2)/2));
                   (sin(Xk_1(3)+Uk(2)/2)) (0.5*Uk(1)*cos(Xk_1(3)+Uk(2)/2));
                   0                    1                                 ];
              P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');
            % De la Medida
            Zk_ = [atan2(LM_xy(laser.id(1),2)-X_k(2),LM_xy(laser.id(1),1)-X_k(1))-X_k(3);
                   atan2(LM_xy(laser.id(2),2)-X_k(2),LM_xy(laser.id(2),1)-X_k(1))-X_k(3);
                   ((LM_xy(laser.id(2),1)-X_k(1))/(atan2(LM_xy(laser.id(2),2)-X_k(2),LM_xy(laser.id(2),1))))];
               
        % Etapa 2: Observación
        Zk = [laser.angle(1);
              laser.angle(2);
              laser.distance(2)];
          
        Hk = [((LM_xy(laser.id(1),2)-X_k(2))/((LM_xy(laser.id(1),1)-X_k(1))^2+(LM_xy(laser.id(1),2)-X_k(2))^2)) (-(LM_xy(laser.id(1),1)-X_k(1))/((LM_xy(laser.id(1),1)-X_k(1))^2+(LM_xy(laser.id(1),2)-X_k(2))^2)) (-1);
              ((LM_xy(laser.id(2),2)-X_k(2))/((LM_xy(laser.id(2),1)-X_k(1))^2+(LM_xy(laser.id(2),2)-X_k(2))^2)) (-(LM_xy(laser.id(2),1)-X_k(1))/((LM_xy(laser.id(2),1)-X_k(1))^2+(LM_xy(laser.id(2),2)-X_k(2))^2)) (-1);   
              ((LM_xy(laser.id(2),2)-X_k(2))/(2*(LM_xy(laser.id(2),1)-X_k(1))*sqrt(1+(LM_xy(laser.id(2),2)-X_k(2))/(LM_xy(laser.id(2),1)-X_k(1))))-(sqrt(1+(LM_xy(laser.id(2),2)-X_k(2))/(LM_xy(laser.id(2),1)-X_k(1))))) (-(LM_xy(laser.id(2),1)-X_k(1))/(2*sqrt(1+(LM_xy(laser.id(2),2)-X_k(2))/(LM_xy(laser.id(2),1)-X_k(1))))) (0)];
        
        % Etapa 3: Comparación de la predicción con observación
        Yk = Zk-Zk_;
        for r=1:3
            if Yk(r)>pi
                Yk(r) = Yk(r) - 2*pi;
            end
            if Yk(r)<(-pi)
                Yk(r) = Yk(r) + 2*pi;
            end
        end
        Sk = Hk*P_k*((Hk)') + Rk;
        Wk = P_k*((Hk)')*inv(Sk);
        
        % Etapa 4: Correccion
        Xk = X_k + Wk*Yk;
        Pk = (eye(3)-Wk*Hk)*P_k; 
        
    end    
  
    
% Bucle principal

    % Fija comando de velocidad para seguir trayectoria
    
    % Simula el avance real del robot
    
    % Estima el avance del robot
    
    % Simula las medida de los sensores
    
    % Estima las medida de los sensores
    
    % Corrige la estimación de la posición del robot