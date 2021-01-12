clear, clc

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

% Algoritmo

velocidadL = 0.2; % m/s
velocidadA = 0.2; % rad/s
tiempo = 0.1; % s

for i = 1:500
    Xk_1=Xk;
    Pk_1=Pk;
    [Xrealk,Xk,Pk]=funcion_EKF(velocidadL,velocidadA,tiempo ,Xk_1,Pk_1);
    Xreal(i,:) = Xrealk;
    Xestimado(i,:) = Xk;
    Pacumulado(1,i) = Pk(1,1);
    Pacumulado(2,i) = Pk(2,2);
    Pacumulado(3,i) = Pk(3,3);
end

figure(1);
subplot(2,2,1);
plot(Xreal(:,1),Xreal(:,2),'.b','MarkerSize',2);
xlabel('X (m)');
ylabel('Y (m)');
hold on;
plot(Xestimado(:,1),Xestimado(:,2),'.r','MarkerSize',2);
legend('Posicion real','Posicion estimada');

subplot(2,2,2);
plot(Xreal(:,1),'.b','MarkerSize',2);
xlabel ('t (muestras)');
ylabel ('X (m)');
hold on;
plot(Xestimado(:,1),'.r','MarkerSize',2);
legend('Posicion real','Posicion estimada');

subplot(2,2,3);
plot(Xreal(:,2),'.b','MarkerSize',2);
xlabel ('t (muestras)');
ylabel ('Y (m)');
hold on;
plot(Xestimado(:,2),'.r','MarkerSize',2);
legend('Posicion real','Posicion estimada');

subplot(2,2,4);
plot(Xreal(:,3),'.b','MarkerSize',2);
xlabel ('t (muestras)');
ylabel ('\theta (rad)');
hold on;
plot(Xestimado(:,3),'.r','MarkerSize',2);
legend('Posicion real','Posicion estimada');



figure(2);
subplot(3,1,1);
axis([0 12 0 9])
plot(Pacumulado(1,:),'b','MarkerSize',2);
xlabel ('t (muestras)')
ylabel ('Varianza X (m2)')
hold on

subplot(3,1,2);
axis([0 12 0 9])
plot(Pacumulado(2,:),'b');
xlabel ('t (muestras)')
ylabel ('Varianza Y (m2)')

subplot(3,1,3);
axis([0 12 0 9])
plot(Pacumulado(3,:),'b');
xlabel ('t (muestras)')
ylabel ('Varianza \theta (rad2)')

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

















