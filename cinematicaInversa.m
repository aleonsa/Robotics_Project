function q = cinematicaInversa(posicion, parametros)
% CINEMATICAINVERSA Calcula los ángulos de las articulaciones dada la posición del efector final
%
%   q = cinematicaInversa(posicion, parametros)
%
%   Entradas:
%   - posicion: Vector [x, y, z] con la posición del efector final
%   - parametros: Estructura con parámetros del robot (l1, l2, l3)
%
%   Salidas:
%   - q: Vector [q1, q2, q3] con las posiciones articulares en radianes

    % Extraer parámetros
    l1 = parametros.l1;
    l2 = parametros.l2;
    l3 = parametros.l3;
    
    % Extraer coordenadas del efector final
    xc = posicion(1);
    yc = posicion(2);
    zc = posicion(3);
    
    % Calcular q1 (ángulo de la primera articulación)
    q1 = atan2(yc, xc);
    
    % Calcular distancia r en el plano XY
    r = sqrt(xc^2 + yc^2);
    
    % Calcular el parámetro D para la solución de q3
    D = (r^2 + (zc-l1)^2 - l2^2 - l3^2) / (2 * l2 * l3);
    
    % Verificar si el punto está dentro del espacio de trabajo
    if abs(D) > 1
        error('Posición (%f, %f, %f) fuera del espacio de trabajo del robot', xc, yc, zc);
    end
    
    % Calcular q3 (solución codo abajo, signo negativo en la raíz)
    q3 = atan2(-sqrt(1 - D^2), D);
    
    % Calcular q2
    s = zc - l1;  % Ajuste para considerar la altura de la base
    q2 = atan2(s, r) - atan2(l3 * sin(q3), l2 + l3 * cos(q3));
    
    % Devolver el vector de ángulos
    q = [q1, q2, q3];
    
    % luego implementar limites del robot ...
end