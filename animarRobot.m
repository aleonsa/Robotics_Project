function animarRobot(q, t, opciones)
% ANIMARROBOT Anima el movimiento de un robot a lo largo de una trayectoria

    parametros = struct('l1', 0, 'l2', 0.1345, 'l3', 0.178);
    
    % Opciones por defecto
    if nargin < 3
        opciones = struct();
    end
    
    if ~isfield(opciones, 'color_trayectoria')
        opciones.color_trayectoria = 'm';
    end
    if ~isfield(opciones, 'mostrar_ejes')
        opciones.mostrar_ejes = true;
    end
    
    % Crear figura optimizada
    figure('Name', 'Animación del Robot', 'NumberTitle', 'off');
    hold on; grid on; view(3); axis equal;
    
    if opciones.mostrar_ejes
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('Trayectoria del Robot');
    end
    
    % Calcular trayectoria del efector final
    num_puntos = size(q, 1);
    trayectoria = zeros(num_puntos, 3);
    
    for i = 1:num_puntos
        [~, pos] = cinematicaDirecta(q(i, :), parametros);
        trayectoria(i, :) = pos;
    end
    
    % Límites de ejes
    margen = 0.35;
    min_xyz = min(trayectoria) - margen;
    max_xyz = max(trayectoria) + margen;
    axis([min_xyz(1) max_xyz(1) min_xyz(2) max_xyz(2) min_xyz(3) max_xyz(3)]);
    
    % DRÁSTICAMENTE reducir frames: solo 15-20 fps
    duracion_total = t(end) - t(1);
    fps = 15;  % 15 fps
    num_frames = round(duracion_total * fps);
    
    % Seleccionar índices uniformemente distribuidos
    indices = round(linspace(1, num_puntos, min(num_frames, num_puntos)));
    tiempo_frame = duracion_total / length(indices);
    
    % Animación con timing preciso
    tiempo_inicio = tic;
    
    for k = 1:length(indices)
        % Tiempo objetivo para este frame
        tiempo_objetivo = (k-1) * tiempo_frame;
        
        % Esperar hasta el momento correcto
        while toc(tiempo_inicio) < tiempo_objetivo
            % Espera activa muy corta
            pause(0.001);
        end
        
        i = indices(k);
        
        % Dibujo rápido
        cla;
        dibujarRobot(q(i, :), parametros);
        plot3(trayectoria(1:i,1), trayectoria(1:i,2), trayectoria(1:i,3), ...
            'Color', opciones.color_trayectoria, 'LineWidth', 2);
        axis([min_xyz(1) max_xyz(1) min_xyz(2) max_xyz(2) min_xyz(3) max_xyz(3)]);
        
        drawnow limitrate;
    end
    
    fprintf('Animación completada en %.2f segundos (simulación: %.2f segundos)\n', ...
            toc(tiempo_inicio), duracion_total);
end