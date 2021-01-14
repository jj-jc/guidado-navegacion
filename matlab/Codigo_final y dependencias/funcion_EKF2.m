function [Xrealk,Xk,Pk]=funcion_EKF2(Xk_1,Pk_1)

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
          0 0 var_dist];

    % Posición de las balizas:
    LM_xy = [ -3.9 -3.0;    % LM1
               4.0  7.9;   % LM2
               4.0 -7.9;   % LM3
               4.0  0.3;   % LM4
               4.0 -0.1;   % LM5
              -3.9  7.9;   % LM6
              -3.9 -7.9;   % LM7
              -3.9  3.0;   % LM8
               7.9 -0.1;   % LM9
               7.9  0.3];  % LM10 






% Se simula el avance del robot con Apolo
    pose = apoloGetLocationMRobot('Pioneer3AT');
    Xrealk = [pose(1) pose(2) pose(4)];
%     Xreal(i,:) = Xrealk;
    
    % Detección de balizas
    laser = apoloGetLaserLandMarks('LMS100');
    num_balizas = length(laser.id); % numero de balizas detectadas
    % Nuevo ciclo: k-1 = k
%             Xk_1 = Xk;
        % Xk_1 = X_k;
%             Pk_1 = Pk;

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
    if num_balizas > 1 % Aplicar filtro cuando se detecten 3 balizas mínimo                
        % De la Medida
        Dx1=LM_xy(laser.id(1),1)-X_k(1);
        Dy1=LM_xy(laser.id(1),2)-X_k(2);

        Dx2=LM_xy(laser.id(2),1)-X_k(1);
        Dy2=LM_xy(laser.id(2),2)-X_k(2);


        Fsin=sin(atan2(Dy2,Dx2));
        Fcos=cos(atan2(Dy2,Dx2));
        Fatan=1/(1+(Dy2/Dx2)^2);

        Zk_ = [atan2(Dy1,Dx1)-X_k(3);
               atan2(Dy2,Dx2)-X_k(3);
               Dy2/Fsin];


        % Etapa 2: Observación
        Zk = [laser.angle(1);
              laser.angle(2);
              laser.distance(2)];

        Hk = [((Dy1)/((Dx1)^2+(Dy1)^2)) (-(Dx1)/((Dx1)^2+(Dy1)^2)) (-1);
              ((Dy2)/((Dx2)^2+(Dy2)^2)) (-(Dx2)/((Dx2)^2+(Dy2)^2)) (-1);   
              ((-Dy2)*(Fcos)*Fatan*(Dy2/(Dx2^2))/Fsin^2) (((-Fsin)-(Dy2*(Fcos)*Fatan*(-1/Dx2)))/Fsin^2) (0) ];

        % Etapa 3: Comparación de la predicción con observación
        Yk = Zk-Zk_;
%       if (Yk(1)>1)||(Yk(2)>1)||(Yk(3)>0.5)
%             Xk=Xk_1;
%             Pk=Pk_1;  
%             
%       else 
           for r=1:3
                if Yk(r)>pi
                    Yk(r) = Yk(r) - 2*pi;
                end
                if Yk(r)<(-pi)
                    Yk(r) = Yk(r) + 2*pi;
                end
            end
            Yk;
            Sk = Hk*P_k*((Hk)') + Rk;
            Wk = P_k*((Hk)')*inv(Sk);

            % Etapa 4: Correccion
            Xk = X_k + Wk*Yk;
            Pk = (eye(3)-Wk*Hk)*P_k;
%         end    
    else 
        Xk=X_k;
        Pk=Pk_1;
    end
    apoloResetOdometry('Pioneer3AT', [Xk(1) Xk(2) Xk(3)]);
end