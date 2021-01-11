clc, clear
% Algoritmo de localización (estimación del estado)
% Filtro de Kalman

% Inicialización del filtro:
    
    % Posición inicial
    x_ini = -2; y_ini = 0; theta_ini = 0;
    apoloPlaceMRobot('Pioneer3AT',[x_ini y_ini 0],theta_ini);
    apoloUpdate();
    apoloResetOdometry('Pioneer3AT', [x_ini y_ini theta_ini]);

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
      
    % Varianza del ruido del proceso
    var_avance = 1.8389e-06;
    var_giro = 2.1857e-07;
    Qk = [var_avance 0;
          0 var_giro];

    % Varianza en la medida (calibración del laser)
    var_dist = 0.00028562;
    var_ang = 0.0001961;
    
    % Inicialización de matriz R 
    Rk = [var_ang 0 0;
          0 var_ang 0;
          0 0 var_ang];

    % Posición de las balizas:
    LM_xy = [ -3.9 -3.0;    % LM1
               4.0  7.9;   % LM2
               4.0 -7.9;   % LM3
               4.0  0.1;   % LM4
               4.0 -0.1;   % LM5
              -3.9  7.9;   % LM6
              -3.9 -7.9;   % LM7
              -3.9  3.0;   % LM8
               7.9 -0.1;   % LM9
               7.9  0.1];  % LM10
              
    
% Algoritmo

velocidadL = 0.2; % m/s
velocidadA = 0.2; % rad/s
tiempo = 0.1; % s

for i = 1:500
    % Se simula el avance del robot con Apolo
    apoloMoveMRobot('Pioneer3AT',[velocidadL, velocidadA], tiempo);
    apoloUpdate();
    pose = apoloGetLocationMRobot('Pioneer3AT');
    Xrealk = [pose(1) pose(2) pose(4)];
    Xreal(i,:) = Xrealk;
    
    % Detección de balizas
    laser = apoloGetLaserLandMarks('LMS100');
    num_balizas = length(laser.id); % numero de balizas detectadas
    if num_balizas > 2 % Aplicar filtro cuando se detecten 3 balizas mínimo
        % Nuevo ciclo: k-1 = k
            Xk_1 = Xk;
            % Xk_1 = X_k;
            Pk_1 = Pk;
        
        % Etapa 1: Predicción
            % Del estado
                % Estimación del avance y giro
                avance_y_giro = apoloGetOdometry('Pioneer3AT');
                avance = sqrt((Xk_1(1)-avance_y_giro(1))^2+(Xk_1(2)-avance_y_giro(2))^2);
                giro = avance_y_giro(3)-Xk_1(3);
            X_k = [(Xk_1(1) + avance*cos(Xk_1(3)+(giro/2)));
                   (Xk_1(2) + avance*sin(Xk_1(3)+(giro/2)));
                   (Xk_1(3) + giro)];
            %apoloResetOdometry('Pioneer3AT', [X_k(1) X_k(2) X_k(3)]);
            Ak = [1 0 (-avance*sin(Xk_1(3)+giro/2));
                  0 1 (avance*cos(Xk_1(3)+giro/2));
                  0 0 1];
            Bk = [(cos(Xk_1(3)+giro/2)) (-0.5*avance*sin(Xk_1(3)+giro/2));
                  (sin(Xk_1(3)+giro/2)) (0.5*avance*cos(Xk_1(3)+giro/2));
                   0 1];
            P_k = Ak*Pk_1*((Ak)') + Bk*Qk*((Bk)');
                
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
              ((LM_xy(laser.id(2),2)-X_k(2))/(2*(LM_xy(laser.id(2),1)-X_k(1))*sqrt(1+abs((LM_xy(laser.id(2),2)-X_k(2))/(LM_xy(laser.id(2),1)-X_k(1)))))-(sqrt(1+abs((LM_xy(laser.id(2),2)-X_k(2))/(LM_xy(laser.id(2),1)-X_k(1)))))) (-(LM_xy(laser.id(2),1)-X_k(1))/(2*sqrt(1+abs((LM_xy(laser.id(2),2)-X_k(2))/(LM_xy(laser.id(2),1)-X_k(1)))))) (0)];
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
        
        apoloResetOdometry('Pioneer3AT', [Xk(1) Xk(2) Xk(3)]);
        
        %Xestimado(i,:) = X_k;
        Xestimado(i,:) = Xk;
        
%         plot(Xestimado(:,1), Xestimado(:,2),'+');
%         xlim([-8 8]);
%         ylim([-8 8]);
        
       
    end
    
end

figure;

plot(Xreal(:,1),Xreal(:,2),'b')
xlabel('X (m)')
ylabel('Y (m)')
hold on;
plot(Xestimado(:,1),Xestimado(:,2),'r')
legend('Posicion real (azul)','Posicion estimada (rojo)')

% figure;
% plot(Xreal(1,:),'b');
% hold on
% plot(Xestimado(1,:),'g');
% xlabel ('t (muestras)')
% ylabel ('X (m)')
% 
% figure
% plot(Xreal(2,:));
% hold on
% plot(Xestimado(2,:),'g');
% xlabel ('t (muestras)')
% ylabel ('Y (m)')

