%% if (grafFlag == 0)
	% {
	% 	grafi[indx][0] = t;
	% 	grafi[indx][1] = q[0];
	% 	grafi[indx][2] = q[1];
	% 	grafi[indx][3] = q[2];
	% 	grafi[indx][4] = taus[0];
	% 	grafi[indx][5] = taus[1];
	% 	grafi[indx][6] = taus[2];
	% 	// grafi[indx][7] = Y[1][0];// qpf[0];
	% 	// grafi[indx][8] = Y[1][1];//qpf[1];
	% 	// grafi[indx][9] = Y[2][1];//qpf[2];
	% 	grafi[indx][7] = qpf[0];
	% 	grafi[indx][8] = qpf[1];
	% 	grafi[indx][9] = qpf[2];
	% 	grafi[indx][10] = qppf[0];
	% 	grafi[indx][11] = qppf[1];
	% 	grafi[indx][12] = qppf[2];
	% 	grafi[indx][13] = qd[0];
	% 	grafi[indx][14] = qd[1];
	% 	grafi[indx][15] = qd[2];

	% 	indx++;
	% 	grafFlag = grafSkip + 1;
	% }

% Load data from CSV file
data = readmatrix('expData_SlotineAdaptable.m', 'FileType', 'text');

% Assign columns to variables for clarity
t      = data(:,1);   % Time
q1     = data(:,2);   % Joint 1 position
q2     = data(:,3);   % Joint 2 position
q3     = data(:,4);   % Joint 3 position
tau1   = data(:,5);   % Control input 1
tau2   = data(:,6);   % Control input 2
tau3   = data(:,7);   % Control input 3
qpf1   = data(:,8);   % Joint 1 velocity
qpf2   = data(:,9);   % Joint 2 velocity
qpf3   = data(:,10);  % Joint 3 velocity
qppf1  = data(:,11);  % Joint 1 acceleration
qppf2  = data(:,12);  % Joint 2 acceleration
qppf3  = data(:,13);  % Joint 3 acceleration
qd1    = data(:,14);  % Desired joint 1 position
qd2    = data(:,15);  % Desired joint 2 position
qd3    = data(:,16);  % Desired joint 3 position

% Derivar las posiciones deseadas
qd1p = gradient(qd1,t);
qd2p = gradient(qd2,t);
qd3p = gradient(qd3,t);

% Derivar las velocidades deseadas
qd1pp = gradient(qd1p,t);
qd2pp = gradient(qd2p,t);
qd3pp = gradient(qd3p,t);

% Convertir a grados
q1_deg  = q1 * 180/pi;
q2_deg  = q2 * 180/pi;
q3_deg  = q3 * 180/pi;
qd1_deg = qd1 * 180/pi;
qd2_deg = qd2 * 180/pi;
qd3_deg = qd3 * 180/pi;

% Cortar los datos a los primeros 10 segundos
idx_10s = t <= 10;
t_plot = t(idx_10s);
q1_deg_plot = q1_deg(idx_10s);
q2_deg_plot = q2_deg(idx_10s);
q3_deg_plot = q3_deg(idx_10s);
qd1_deg_plot = qd1_deg(idx_10s);
qd2_deg_plot = qd2_deg(idx_10s);
qd3_deg_plot = qd3_deg(idx_10s);
tau1_plot = tau1(idx_10s);
tau2_plot = tau2(idx_10s);
tau3_plot = tau3(idx_10s);

%% Plots
% ------------ Plots de seguimiento de trayectoria y error (grados)  ------------
figure('Name','Seguimiento de trayectoria y error (grados)','Position',[100 100 1000 800]);
subtitle('Seguimiento de trayectoria y error ');

% Articulación 1
subplot(3,2,1);
plot(t_plot, qd1_deg_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, q1_deg_plot, 'b', 'LineWidth', 1.2);
ylabel('q_1 (°)');
legend('Referencia','Medida');
title('Articulación 1: Ref vs Medida');
grid on;

subplot(3,2,2);
plot(t_plot, qd1_deg_plot - q1_deg_plot, 'k', 'LineWidth', 1.2);
ylabel('Error (°)');
title('Error de seguimiento q_1');
grid on;

% Articulación 2
subplot(3,2,3);
plot(t_plot, qd2_deg_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, q2_deg_plot, 'b', 'LineWidth', 1.2);
ylabel('q_2 (°)');
legend('Referencia','Medida');
title('Articulación 2: Ref vs Medida');
grid on;

subplot(3,2,4);
plot(t_plot, qd2_deg_plot - q2_deg_plot, 'k', 'LineWidth', 1.2);
ylabel('Error (°)');
title('Error de seguimiento q_2');
grid on;

% Articulación 3
subplot(3,2,5);
plot(t_plot, qd3_deg_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, q3_deg_plot, 'b', 'LineWidth', 1.2);
xlabel('Tiempo (s)');
ylabel('q_3 (°)');
legend('Referencia','Medida');
title('Articulación 3: Ref vs Medida');
grid on;

subplot(3,2,6);
plot(t_plot, qd3_deg_plot - q3_deg_plot, 'k', 'LineWidth', 1.2);
xlabel('Tiempo (s)');
ylabel('Error (°)');
title('Error de seguimiento q_3');
grid on;

% ---------------- Plots de señales de control (taus) -------------------
figure('Name','Señales de control (taus)','Position',[200 200 900 600]);
subtitle('Señales de control ');

subplot(3,1,1);
plot(t_plot, tau1_plot, 'LineWidth', 1.5);
ylabel('\tau_1 (Nm)');
title('Señal de control \tau_1');
grid on;

subplot(3,1,2);
plot(t_plot, tau2_plot, 'LineWidth', 1.5);
ylabel('\tau_2 (Nm)');
title('Señal de control \tau_2');
grid on;

subplot(3,1,3);
plot(t_plot, tau3_plot, 'LineWidth', 1.5);
ylabel('\tau_3 (Nm)');
xlabel('Tiempo (s)');
title('Señal de control \tau_3');
grid on;

% --- Cálculo de velocidades y aceleraciones de referencia usando polinomio de 5to orden ---
tf = 2.0;
qf_deg = [50; 50; -60]; % grados
qf = qf_deg * pi/180;   % radianes

a0 = [q1(1); q2(1); q3(1)];
a3 = 10 * (qf - a0) / tf^3;
a4 = -15 * (qf - a0) / tf^4;
a5 = 6 * (qf - a0) / tf^5;

% Prealocar
qd1p_poly = zeros(size(t)); qd2p_poly = zeros(size(t)); qd3p_poly = zeros(size(t));
qd1pp_poly = zeros(size(t)); qd2pp_poly = zeros(size(t)); qd3pp_poly = zeros(size(t));

for k = 1:length(t)
    if t(k) <= tf
        qd1p_poly(k) = 3*a3(1)*t(k)^2 + 4*a4(1)*t(k)^3 + 5*a5(1)*t(k)^4;
        qd2p_poly(k) = 3*a3(2)*t(k)^2 + 4*a4(2)*t(k)^3 + 5*a5(2)*t(k)^4;
        qd3p_poly(k) = 3*a3(3)*t(k)^2 + 4*a4(3)*t(k)^3 + 5*a5(3)*t(k)^4;
        qd1pp_poly(k) = 6*a3(1)*t(k) + 12*a4(1)*t(k)^2 + 20*a5(1)*t(k)^3;
        qd2pp_poly(k) = 6*a3(2)*t(k) + 12*a4(2)*t(k)^2 + 20*a5(2)*t(k)^3;
        qd3pp_poly(k) = 6*a3(3)*t(k) + 12*a4(3)*t(k)^2 + 20*a5(3)*t(k)^3;
    else
        qd1p_poly(k) = 0; qd2p_poly(k) = 0; qd3p_poly(k) = 0;
        qd1pp_poly(k) = 0; qd2pp_poly(k) = 0; qd3pp_poly(k) = 0;
    end
end

% Convertir a grados
qd1p_poly_deg = qd1p_poly * 180/pi;
qd2p_poly_deg = qd2p_poly * 180/pi;
qd3p_poly_deg = qd3p_poly * 180/pi;
qd1pp_poly_deg = qd1pp_poly * 180/pi;
qd2pp_poly_deg = qd2pp_poly * 180/pi;
qd3pp_poly_deg = qd3pp_poly * 180/pi;

% Cortar a los primeros 10 segundos
qd1p_poly_plot = qd1p_poly_deg(idx_10s);
qd2p_poly_plot = qd2p_poly_deg(idx_10s);
qd3p_poly_plot = qd3p_poly_deg(idx_10s);
qd1pp_poly_plot = qd1pp_poly_deg(idx_10s);
qd2pp_poly_plot = qd2pp_poly_deg(idx_10s);
qd3pp_poly_plot = qd3pp_poly_deg(idx_10s);

% Velocidades medidas en grados
qpf1_plot = qpf1(idx_10s) * 180/pi;
qpf2_plot = qpf2(idx_10s) * 180/pi;
qpf3_plot = qpf3(idx_10s) * 180/pi;

% --- Plots: columna izquierda velocidad medida vs deseada, derecha aceleración medida vs deseada ---
figure('Name','Velocidades y aceleraciones','Position',[400 400 900 700]);
subtitle('Velocidades y aceleraciones (medida vs ref, primeros 10 s)');

subplot(3,2,1);
plot(t_plot, qd1p_poly_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, qpf1_plot, 'b', 'LineWidth', 1.2);
ylabel('q_1'' (°/s)');
legend('Referencia','Medida');
title('Velocidad Articulación 1');
grid on;

subplot(3,2,2);
plot(t_plot, qd1pp_poly_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, qppf1(idx_10s) * 180/pi, 'b', 'LineWidth', 1.2);
ylabel('q_1'''' (°/s²)');
legend('Referencia','Medida');
title('Aceleración Articulación 1');
grid on;

subplot(3,2,3);
plot(t_plot, qd2p_poly_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, qpf2_plot, 'b', 'LineWidth', 1.2);
ylabel('q_2'' (°/s)');
legend('Referencia','Medida');
title('Velocidad Articulación 2');
grid on;

subplot(3,2,4);
plot(t_plot, qd2pp_poly_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, qppf2(idx_10s) * 180/pi, 'b', 'LineWidth', 1.2);
ylabel('q_2'''' (°/s²)');
legend('Referencia','Medida');
title('Aceleración Articulación 2');
grid on;

subplot(3,2,5);
plot(t_plot, qd3p_poly_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, qpf3_plot, 'b', 'LineWidth', 1.2);
xlabel('Tiempo (s)');
ylabel('q_3'' (°/s)');
legend('Referencia','Medida');
title('Velocidad Articulación 3');
grid on;

subplot(3,2,6);
plot(t_plot, qd3pp_poly_plot, 'r--', 'LineWidth', 1.5); hold on;
plot(t_plot, qppf3(idx_10s) * 180/pi, 'b', 'LineWidth', 1.2);
xlabel('Tiempo (s)');
ylabel('q_3'''' (°/s²)');
legend('Referencia','Medida');
title('Aceleración Articulación 3');
grid on;

