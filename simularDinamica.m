function [t, estados, u, Theta_t] = simularDinamica(qd, qd_dot, qd_ddot, tiempo_sim, Ts, delta, controlador)
% SIMULARDINAMICA Simula la dinámica del robot con un controlador específico
%
%   [t, estados, u] = simularDinamica(qd, qd_dot, qd_ddot, tiempo_sim, Ts, controlador)
%
%   Entradas:
%   - qd: Trayectoria deseada de posición [num_puntos x 3]
%   - qd_dot: Trayectoria deseada de velocidad [num_puntos x 3]
%   - qd_ddot: Trayectoria deseada de aceleración [num_puntos x 3]
%   - tiempo_sim: Tiempo total de simulación en segundos
%   - Ts: Período de muestreo en segundos
%   - delta: incertidumbre paramétrica
%   - controlador: Estructura que define el controlador a utilizar
%       - tipo: Tipo de controlador ('PD', 'PID', 'Computed Torque', etc.)
%       - parametros: Estructura con parámetros específicos del controlador
%
%   Entradas adicionales:
%   - opciones_ruido: Estructura con opciones de ruido
%   - ruido_medicion: Desviación estándar del ruido en las mediciones
%   - ruido_actuador: Desviación estándar del ruido en los actuadores
%   - incertidumbre_parametros: Factores de incertidumbre en los parámetros
%
%   Salidas:
%   - t: Vector de tiempos
%   - estados: Matriz con estados del sistema [posición; velocidad] (6 x num_puntos)
%   - u: Matriz con señales de control (3 x num_puntos)

    % Crear vector de tiempo
    t = 0:Ts:tiempo_sim;
    num_puntos = length(t);
    
    % Inicializar matrices para almacenar resultados
    e_int = 0;
    estados = zeros(6, num_puntos);
    u = zeros(3, num_puntos);
    Theta_t = zeros(8, num_puntos);
    
    % Condiciones iniciales
    q = zeros(3, 1);      % Posiciones iniciales (puede ser modificado según se requiera)
    q_dot = zeros(3, 1);  % Velocidades iniciales
    
    % Vector de estado x = [q; q_dot]
    x = [q; q_dot];

    th1 = 0.002517290542366;
    th2 = 0.001082466154947;
    th3 = 0.001374082981419;
    th4 = 0.000768230130927;
    th5 = 0.035267357329092;
    th6 = 0.007444737668751;
    th7 = 0.004491588558515;
    th8 = 0.005345056038429;
    Theta_anterior = [th1, th2, th3, th4, th5, th6, th7, th8]';
    
    % Simulación de la dinámica del sistema
    for i = 1:num_puntos
        % Extraer posición y velocidad actuales
        q = x(1:3);
        q_dot = x(4:6);
        
        % Obtener la referencia deseada para este instante
        q_des = qd(i, :)';
        q_dot_des = qd_dot(i, :)';
        q_ddot_des = qd_ddot(i, :)';
        
        % Calcular matrices dinámicas
        [H, C, D, G] = dinamicaRobot(q, q_dot, 0);
        
        % Calcular errores
        e = q - q_des;
        e_dot = q_dot - q_dot_des;
        
        Kp = blkdiag(2,2.3,2);
        Kd = blkdiag(0.03,0.03,0.03);

        % Calcular la señal de control según el tipo de controlador
        switch controlador
            case 'PD'
                % Controlador PD con compensación de gravedad
                u(:,i) = - Kp * e - Kd * e_dot + G;
                
            case 'PID'
                Ki = blkdiag(0.1, 0.1, 0.1);
                % Actualizar error integral
                e_int = e_int + e * Ts;
                
                % Controlador PID con compensación de gravedad
                u(:,i) = - Kp*e - Ki*e_int - Kd*e_dot + G;
                
            case 'ComputedTorque'
                Kp = 100*Kp;
                Kd = 100*Kd;
                % Controlador por par calculado
                u(:,i) = H * (q_ddot_des - Kp*e - Kd*e_dot) + C*q_dot + D*q_dot + G;
            
            case 'Slotine'
                Kp = 1*Kp;
                Kd = 1*Kd;
                Lambda = (Kd\Kp); % Lambda = inv(Kd) * Kp
            
                % Señales de referencia
                qr_dot = q_dot_des - Lambda*e;
                qr_ddot = q_ddot_des - Lambda*e_dot;
            
                % Variable de superficie
                s = q_dot - qr_dot;
            
                % Ley de control
                u(:,i) = H*qr_ddot + C*qr_dot + D*qr_dot + G - Kd*s;
              
            case 'SlotineAdaptable'
                Kp = 1*Kp;
                Kd = 1*Kd;
                Lambda = (Kd\Kp); % Lambda = inv(Kd) * Kp

                 % Señales de referencia
                qr_dot = q_dot_des - Lambda*e;
                qr_ddot = q_ddot_des - Lambda*e_dot;
                % Variable de superficie    
                s = q_dot - qr_dot;
                
                Theta = identificacion(q,q_dot,qr_ddot,s,Ts, Theta_anterior);
                Theta_anterior = Theta;
                [Hhat, Chat, Dhat, Ghat] = dinamicaRobot(q, q_dot, 0, Theta);

                % Ley de control
                u(:,i) = Hhat*qr_ddot + Chat*qr_dot + Dhat*qr_dot + Ghat - Kd*s;

%                 plot(i,Theta);
%                 hold on;
                Theta_t(:,i) = Theta;

            otherwise
                error('Tipo de controlador no soportado: %s', controlador);
        end
        
        % Respuesta del sistema (calcular aceleración)
        q_ddot = inv(H) * (u(:,i) - C * q_dot - D * q_dot - G);
        
        % Integrador Runge-Kutta de orden 4
        x_old = x;
        
        k1 = [q_dot; q_ddot];
        
        % Actualizar posición y velocidad temporales para k2
        q_temp = q + k1(1:3) * Ts/2;
        q_dot_temp = q_dot + k1(4:6) * Ts/2;
        [H_temp, C_temp, D_temp, G_temp] = dinamicaRobot(q_temp, q_dot_temp, delta);
        tau_temp = u(:,i);  % Usamos el último control calculado
        q_ddot_temp = inv(H_temp) * (tau_temp - C_temp * q_dot_temp - D_temp * q_dot_temp - G_temp);
        k2 = [q_dot_temp; q_ddot_temp];
        
        % Actualizar posición y velocidad temporales para k3
        q_temp = q + k2(1:3) * Ts/2;
        q_dot_temp = q_dot + k2(4:6) * Ts/2;
        [H_temp, C_temp, D_temp, G_temp] = dinamicaRobot(q_temp, q_dot_temp, delta);
        tau_temp = u(:,i);  % Usamos el último control calculado
        q_ddot_temp = inv(H_temp) * (tau_temp - C_temp * q_dot_temp - D_temp * q_dot_temp - G_temp);
        k3 = [q_dot_temp; q_ddot_temp];
        
        % Actualizar posición y velocidad temporales para k4
        q_temp = q + k3(1:3) * Ts;
        q_dot_temp = q_dot + k3(4:6) * Ts;
        [H_temp, C_temp, D_temp, G_temp] = dinamicaRobot(q_temp, q_dot_temp, delta);
        tau_temp = u(:,i);  % Usamos el último control calculado
        q_ddot_temp = inv(H_temp) * (tau_temp - C_temp * q_dot_temp - D_temp * q_dot_temp - G_temp);
        k4 = [q_dot_temp; q_ddot_temp];
        
        % Actualizar el vector de estado
        x = x_old + (1/6) * Ts * (k1 + 2*k2 + 2*k3 + k4);
        
        % Almacenar estados
        estados(:,i) = x;
    end
end