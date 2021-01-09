
% Algoritmo de localización (estimación del estado)
% Filtro de Kalman

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
            % De la Medida
            Zk_ = [-X_k(3)+pi/2+atan2(LM_xy(laser.id(1),1)-X_k(1),LM_xy(laser.id(1),2));
                   -X_k(3)+pi/2+atan2(LM_xy(laser.id(2),1)-X_k(1),LM_xy(laser.id(2),2));
                   -X_k(3)+pi/2+atan2(LM_xy(laser.id(3),1)-X_k(1),LM_xy(laser.id(3),2))];
               
        % Etapa 2: Observación
        Zk = [laser.angle(1);
              laser.angle(2);
              laser.angle(3)];
        Hk = [((LM_xy(laser.id(1),2)-X_k(2))/((LM_xy(laser.id(1),1)-X_k(1))^2+(LM_xy(laser.id(1),2)-X_k(2))^2)) (-(LM_xy(laser.id(1),1)-X_k(1))/((LM_xy(laser.id(1),1)-X_k(1))^2+(LM_xy(laser.id(1),2)-X_k(2))^2)) (-1);
              ((LM_xy(laser.id(2),2)-X_k(2))/((LM_xy(laser.id(2),1)-X_k(1))^2+(LM_xy(laser.id(2),2)-X_k(2))^2)) (-(LM_xy(laser.id(2),1)-X_k(1))/((LM_xy(laser.id(2),1)-X_k(1))^2+(LM_xy(laser.id(2),2)-X_k(2))^2)) (-1);   
              ((LM_xy(laser.id(3),2)-X_k(2))/((LM_xy(laser.id(3),1)-X_k(1))^2+(LM_xy(laser.id(3),2)-X_k(2))^2)) (-(LM_xy(laser.id(3),1)-X_k(1))/((LM_xy(laser.id(3),1)-X_k(1))^2+(LM_xy(laser.id(3),2)-X_k(2))^2)) (-1)];
        
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