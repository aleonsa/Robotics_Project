function [H, C, D, G] = dinamicaRobot(q, qdot,delta, Theta)
% DINAMICAROBOT Calcula las matrices de la dinámica del robot 
% tau = M*qddot + C*qdot + D*qdot + G 
%
%   [H, C, D, G] = dinamicaRobot(q, qdot)
%
%   Entradas:
%   - q: Vector con posiciones articulares [q1, q2, q3]
%   - qdot: Vector con velocidades articulares [qdot1, qdot2, qdot3]
%   - delta: Incertidumbre
%
%   Salidas:
%   - H: Matriz de inercia (3x3)
%   - C: Matriz de fuerzas centrífugas y de Coriolis (3x3)
%   - D: Matriz de fricciones (3x3)
%   - G: Vector de fuerzas gravitacionales (3x1)

    % Extraer valores de q y qdot
    % q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    qdot1 = qdot(1);
    qdot2 = qdot(2);
    qdot3 = qdot(3);
    
    % Constantes útiles
    % s1 = sin(q1);
    % c1 = cos(q1);
    s2 = sin(q2);
    c2 = cos(q2);
    s3 = sin(q3);
    c3 = cos(q3);
    s23 = sin(q2 + q3);
    c23 = cos(q2 + q3);
    
    % Gravedad
    g0 = 9.81;  % Aceleración debida a la gravedad (m/s^2)
    
    if nargin < 4
    % Vector de Parámetros nominales:
        th1_nom = 0.002517290542366;
        th2_nom = 0.001082466154947;
        th3_nom = 0.001374082981419;
        th4_nom =0.000768230130927;
        th5_nom =  0.035267357329092;
        th6_nom = 0.007444737668751;
        th7_nom = 0.004491588558515;
        th8_nom =  0.005345056038429 ;
    else
         th1_nom = Theta(1);
         th2_nom = Theta(2);
         th3_nom = Theta(3);
         th4_nom = Theta(4);
         th5_nom = Theta(5);
         th6_nom = Theta(6);
         th7_nom = Theta(7);
         th8_nom = Theta(8);
    end
    % Theta = [th1, th2, th3, th4, th5, th6, th7, th8]';
%     static double th[p] = { ,    ,    ,  0 * ,  0 *, 0 * ,, };//NEW FOR 1500


    % Agregamos incertidumbre paramétrica variante en el tiempo
    % Usar una variable persistent para simular el tiempo
    persistent contador;
    if isempty(contador)
        contador = 0;
    end
    contador = contador + 1;
    % t_actual = contador * Ts;
    incertidumbre = delta;
    
    % Variables persistent para almacenar los valores de ruido
    persistent ruido_th1 ruido_th2 ruido_th3 ruido_th4 ruido_th5 ruido_th6 ruido_th7 ruido_th8;
    
    % Generar nuevo ruido cada n pasos (ruido lento)
    n = 50;
    if mod(contador, n) == 1 || isempty(ruido_th1)
        ruido_th1 = incertidumbre * (2*rand() - 1);
        ruido_th2 = incertidumbre * (2*rand() - 1);
        ruido_th3 = incertidumbre * (2*rand() - 1);
        ruido_th4 = incertidumbre * (2*rand() - 1);
        ruido_th5 = incertidumbre * (2*rand() - 1);
        ruido_th6 = incertidumbre * (2*rand() - 1);
        ruido_th7 = incertidumbre * (2*rand() - 1);
        ruido_th8 = incertidumbre * (2*rand() - 1);
    end
    
    % Aplicar el ruido lento a los parámetros
    th1 = th1_nom * (1 + ruido_th1);
    th2 = th2_nom * (1 + ruido_th2);
    th3 = th3_nom * (1 + ruido_th3);
    th4 = th4_nom * (1 + ruido_th4);
    th5 = th5_nom * (1 + ruido_th5);
    th6 = th6_nom * (1 + ruido_th6);
    th7 = th7_nom * (1 + ruido_th7);
    th8 = th8_nom * (1 + ruido_th8);

    % Matriz de inercias
    H = zeros(3,3);
    H(1,1) = (c2^2)*th1 + c2*c23*th2 + (s23^2)*th3;
    H(2,2) = th1 + 2*c3*th2 + th3;
    H(2,3) = c3*th2 + th3;
    H(3,2) = H(2,3);
    H(3,3) = th3;
    
    % Vector de Gravedad
    G = zeros(3,1);
    G(2,1) = g0*(c2*th7 + c23*th8);
    G(3,1) = g0*c23*th8;

    % Matriz de Coriolis
    C = zeros(3,3);
    C(1,1) = -c2*s2*qdot2*th1 - 0.5*(c2*s23*(qdot2 + qdot3) + s2*c23*qdot2)*th2 + c23*s23*(qdot2 + qdot3)*th3;
    C(1,2) = -c2*s2*qdot1*th1 - 0.5*(s2*c23 + c2*s23)*qdot1*th2 + s23*c23*qdot1*th3;
    C(1,3) = -0.5*c2*s23*qdot1*th2 + c23*s23*qdot1*th3;
    C(2,1) = c2*s2*qdot1*th1 + 0.5*(s2*c23 + c2*s23)*qdot1*th2 - s23*c23*qdot1*th3;
    C(2,2) = -s3*qdot3*th2;
    C(2,3) = -s3*(qdot2 + qdot3)*th2;
    C(3,1) = 0.5*c2*s23*qdot1*th2 - c23*s23*qdot1*th3;
    C(3,2) = s3*qdot2*th2;
    C(3,3) = 0;

    % Matriz de Fricciones
    D = zeros(3,3);
    D(1,1) = th4;
    D(2,2) = th5;
    D(3,3) = th6;
    
end