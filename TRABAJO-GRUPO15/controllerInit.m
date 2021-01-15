function [controller] = controllerInit(trajectory, vl, va, lookahead)
%Inicialización de los parámetros del algoritmo PurePursuit
controller = controllerPurePursuit;

%Definición de los puntos de la trayectoria
controller.Waypoints = trajectory;

%Velocidad lineal deseada
controller.DesiredLinearVelocity = vl;

%Velocidad angular máxima (Obtenida empíricamente de Apolo)
controller.MaxAngularVelocity = va;

%Parámetro de visión a futuro. Esto hará que los giros sean más o menos
%bruscos
controller.LookaheadDistance = lookahead;

