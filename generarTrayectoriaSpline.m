function [qd, qd_dot, qd_ddot] = generarTrayectoriaSpline(q_inicial, q_final, tiempo_total, tiempo_sim, Ts)
% GENERARTRAYECTORIASPLINE Genera una trayectoria con spline cúbico
%
%   [qd, qd_dot, qd_ddot, t] = generarTrayectoriaSpline(q_inicial, q_final, tiempo_total, Ts)
%
%   Entradas:
%   - q_inicial: Vector con posiciones iniciales [q1_0, q2_0, q3_0]
%   - q_final: Vector con posiciones finales [q1_f, q2_f, q3_f]
%   - tiempo_total: Tiempo total para completar la trayectoria (segundos)
%   - Ts: Período de muestreo (segundos)
%
%   Salidas:
%   - qd: Trayectoria deseada de posición [num_puntos x 3]
%   - qd_dot: Trayectoria deseada de velocidad [num_puntos x 3]
%   - qd_ddot: Trayectoria deseada de aceleración [num_puntos x 3]
%   - t: Vector de tiempos

    % Crear vector de tiempo
    t = 0:Ts:tiempo_total;
    num_puntos = length(t);
    num_puntos_sim = length(0:Ts:tiempo_sim);
    
    % Inicializar matrices de salida
    qd = zeros(num_puntos, 3);
    qd_dot = zeros(num_puntos, 3);
    qd_ddot = zeros(num_puntos, 3);
    
    % Condiciones iniciales y finales (asumiendo velocidades cero en extremos)
    % Posiciones
    q0 = q_inicial;
    qf = q_final;
    
    % Velocidades (iniciales y finales en cero)
    q0_dot = [0, 0, 0];  % Velocidad inicial
    qf_dot = [0, 0, 0];  % Velocidad final
    
    % Calcular coeficientes del spline cúbico para cada articulación
    % Polinomio cúbico: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
    
    for i = 1:3  % Para cada articulación
        % Resolver el sistema de ecuaciones para obtener los coeficientes a0, a1, a2, a3
        % Condiciones:
        % q(0) = q0(i)
        % q(tf) = qf(i)
        % q_dot(0) = q0_dot(i)
        % q_dot(tf) = qf_dot(i)
        
        % Matriz del sistema de ecuaciones
        A = [1,  0,           0,               0;
             1,  tiempo_total, tiempo_total^2, tiempo_total^3;
             0,  1,           0,               0;
             0,  1,           2*tiempo_total,  3*tiempo_total^2];
        
        % Vector de condiciones
        b = [q0(i); qf(i); q0_dot(i); qf_dot(i)];
        
        % Resolver para obtener coeficientes
        coeficientes = A \ b;
        a0 = coeficientes(1);
        a1 = coeficientes(2);
        a2 = coeficientes(3);
        a3 = coeficientes(4);
        
        % Generar la trayectoria para esta articulación
        for j = 1:num_puntos
            % Posición: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
            qd(j, i) = a0 + a1*t(j) + a2*t(j)^2 + a3*t(j)^3;
            
            % Velocidad: q_dot(t) = a1 + 2*a2*t + 3*a3*t^2
            qd_dot(j, i) = a1 + 2*a2*t(j) + 3*a3*t(j)^2;
            
            % Aceleración: q_ddot(t) = 2*a2 + 6*a3*t
            qd_ddot(j, i) = 2*a2 + 6*a3*t(j);
        end
        for j = num_puntos:num_puntos_sim
            qd(j, i) = q_final(i);
            qd_dot(j, i) = 0;
            qd_ddot(j, i) = 0;
        end
    end
end