function [T, posicion] = cinematicaDirecta(q, parametros)
% CINEMATICADIRECTA Calcula la matriz de transformación y posición para un robot
%
%   [T, posicion] = cinematicaDirecta(q, parametros)
%
%   Entradas:
%   - q: Vector con posiciones articulares [q1, q2, q3]
%   - parametros: Estructura con parámetros del robot (l1, l2, l3)
%
%   Salidas:
%   - T: Matriz de transformación homogénea 4x4
%   - posicion: Vector [x, y, z] con la posición del efector final

    % Extraer las posiciones articulares
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    % Extraer parámetros
    l1 = parametros.l1;
    l2 = parametros.l2;
    l3 = parametros.l3;
    
    % Calcular senos y cosenos
    c1 = cos(q1);
    s1 = sin(q1);
    c2 = cos(q2);
    s2 = sin(q2);
    c23 = cos(q2 + q3);
    s23 = sin(q2 + q3);
    
    % Calcular la matriz de transformación homogénea
    T = [c1*c23, -c1*s23, s1, c1*(l2*c2+l3*c23);
         s1*c23, -s1*s23, -c1, s1*(l2*c2+l3*c23);
         s23, c23, 0, l1+l2*s2+l3*s23;
         0, 0, 0, 1];
    
    % Extraer posición del efector final
    posicion = T(1:3, 4)';
end