%% Script para únicamente cinemática

% Definir parámetros del robot
% parametros = struct('l1', 0, 'l2', 0.1345, 'l3', 0.178);
% 
% % Amplitudes y frecuencias para cada articulación
% amplitudes = [pi/6, pi/4, pi/3];  % Amplitudes en radianes
% frecuencias = [1, 2, 3];          % Frecuencias relativas
% duracion = 5;                    % Duración en segundos
% frecuencia_muestreo = 200;         % Muestreo en Hz
% 
% % Generar trayectoria senoidal
% [q, t] = generarTrayectoria(amplitudes, frecuencias, duracion, frecuencia_muestreo);
% 
% % Animar el robot
% animarRobot(q, t, parametros);

%% Script para dinámica
clear; close all; clc;

tiempo_sim = 8;  % Segundos
Ts = 0.01;        % Período de muestreo
t = 0:Ts:tiempo_sim;

% Trayectoria deseada (spline cubico)
q_inicial = [0, 0, 0];           % Posiciones iniciales en radianes
q_final = [pi/3, pi/4, pi/6];    % Posiciones finales en radianes
tiempo_total = 5;                % tiempo en el que llega a q_final
[qd, qd_dot, qd_ddot] = generarTrayectoriaSpline(q_inicial, q_final, tiempo_total, tiempo_sim, Ts);

% controlador = 'PD';
% controlador = 'PID';
% controlador = 'ComputedTorque';
controlador = 'SlotineAdaptable';
delta = 0.1; % incertidumbre paramétrica
[t, estados, u, Theta] = simularDinamica(qd, qd_dot, qd_ddot, tiempo_sim, Ts, delta ,controlador);

q = estados(1:3,:);

%% Visualizar resultados
opciones = struct('color_trayectoria', 'm', ...
                'factor_velocidad', 1000, ...
                'mostrar_ejes', true);
animarRobot(q', t, opciones);

% Gráfico de posiciones articulares
figure;
subplot(3,1,1);
plot(t, q(1,:), 'r-', t, qd(:,1), 'b--', 'LineWidth', 1.5);
ylabel('q1 (rad)');
legend('Real', 'Deseada');
title('Posiciones Articulares');
grid on;

subplot(3,1,2);
plot(t, q(2,:), 'r-', t, qd(:,2), 'b--', 'LineWidth', 1.5);
ylabel('q2 (rad)');
grid on;

subplot(3,1,3);
plot(t, q(3,:), 'r-', t, qd(:,3), 'b--', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('q3 (rad)');
grid on;

% Gráfico de señales de control
figure;
subplot(3,1,1);
plot(t, u(1,:), 'LineWidth', 1.5);
ylabel('tau1 (Nm)');
title('Señales de Control');
grid on;

subplot(3,1,2);
plot(t, u(2,:), 'LineWidth', 1.5);
ylabel('tau2 (Nm)');
grid on;

subplot(3,1,3);
plot(t, u(3,:), 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('tau3 (Nm)');
grid on;



