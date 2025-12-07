%% =================================================================
% SIMULACIÓN PLL: COMPARACIÓN DE FILTROS
% Muestra el error de fase para 3 filtros (RC, Lead-Lag, Phase-Lag)
% ante 3 tipos de entrada (escalón fase, salto frecuencia, rampa)
%% =================================================================
clear; close all; clc;

%% Parámetros del PLL
Kd = 1;         % Ganancia detector de fase (V/rad)
Kv = 1000;      % Ganancia VCO (rad/s por V)
s = tf('s');    % Variable de Laplace

%% Definición de los 3 filtros

% 1) RC simple (primer orden): solo un polo
tau_RC = 1e-3;
F_RC = 1/(1 + s*tau_RC);

% 2) Lead-Lag: CERO a frecuencias MÁS BAJAS que el polo
%    (mejora margen de fase y acelera respuesta)
tau_z_lead = 2e-3;   % Cero en 1/(2e-3) = 500 rad/s
tau_p_lead = 5e-4;   % Polo en 1/(5e-4) = 2000 rad/s
F_leadlag = (1 + s*tau_z_lead) / (1 + s*tau_p_lead);

% 3) Phase-Lag: CERO a frecuencias MÁS ALTAS que el polo
%    (aumenta ganancia DC → reduce error estacionario)
tau_z_lag = 5e-4;    % Cero en 2000 rad/s
tau_p_lag = 5e-3;    % Polo en 200 rad/s
F_phaselag = (1 + s*tau_z_lag) / (1 + s*tau_p_lag);

%% Funciones de transferencia de ERROR: E(s) = 1/(1 + L(s))
% donde L(s) = Kd*Kv*F(s)/s es el lazo abierto
E_RC = 1 / (1 + (Kd*Kv*F_RC)/s);
E_lead = 1 / (1 + (Kd*Kv*F_leadlag)/s);
E_lag = 1 / (1 + (Kd*Kv*F_phaselag)/s);

%% Vector de tiempo
t = 0:1e-5:0.06;  % 0 a 60 ms para ver mejor el régimen permanente

%% ========== GRÁFICA 1: ESCALÓN DE FASE ==========
Delta_phi = 0.1;  % Escalón de 0.1 rad
phi_step = Delta_phi * ones(size(t));

% Calcular error = lsim(E(s), entrada, t)
err_step_RC = lsim(E_RC, phi_step, t);
err_step_lead = lsim(E_lead, phi_step, t);
err_step_lag = lsim(E_lag, phi_step, t);

figure('Position', [100 100 1200 400]); %[output:0940899c]
subplot(1,3,1); %[output:0940899c]
plot(t*1000, err_step_RC, 'r', 'LineWidth', 2); hold on; %[output:0940899c]
plot(t*1000, err_step_lead, 'b', 'LineWidth', 2); %[output:0940899c]
plot(t*1000, err_step_lag, 'g', 'LineWidth', 2); %[output:0940899c]
grid on; %[output:0940899c]
xlabel('Tiempo (ms)'); %[output:0940899c]
ylabel('Error (rad)'); %[output:0940899c]
title('Error ante Escalón de Fase'); %[output:0940899c]
legend('RC', 'Lead-Lag', 'Phase-Lag', 'Location', 'best'); %[output:0940899c]

%% ========== GRÁFICA 2: SALTO DE FRECUENCIA ==========
% Un salto de frecuencia Δf produce una fase φ(t) = 2π*Δf*t (rampa)
Delta_f = 2;  % Salto de 2 Hz
phi_freq = 2*pi*Delta_f * t;

err_freq_RC = lsim(E_RC, phi_freq, t);
err_freq_lead = lsim(E_lead, phi_freq, t);
err_freq_lag = lsim(E_lag, phi_freq, t);

subplot(1,3,2); %[output:0940899c]
plot(t*1000, err_freq_RC, 'r', 'LineWidth', 2); hold on; %[output:0940899c]
plot(t*1000, err_freq_lead, 'b', 'LineWidth', 2); %[output:0940899c]
plot(t*1000, err_freq_lag, 'g', 'LineWidth', 2); %[output:0940899c]
grid on; %[output:0940899c]
xlabel('Tiempo (ms)'); %[output:0940899c]
ylabel('Error (rad)'); %[output:0940899c]
title('Error ante Salto de Frecuencia'); %[output:0940899c]
legend('RC', 'Lead-Lag', 'Phase-Lag', 'Location', 'best'); %[output:0940899c]

%% ========== GRÁFICA 3: RAMPA DE FRECUENCIA ==========
% Una rampa de frecuencia a*t produce fase φ(t) = π*a*t² (parábola)
a_f = 50;  % Rampa de 50 Hz/s (aumentado para ver efecto más claro)
phi_ramp = pi*a_f * t.^2;

err_ramp_RC = lsim(E_RC, phi_ramp, t);
err_ramp_lead = lsim(E_lead, phi_ramp, t);
err_ramp_lag = lsim(E_lag, phi_ramp, t);

subplot(1,3,3); %[output:0940899c]
plot(t*1000, err_ramp_RC, 'r', 'LineWidth', 2); hold on; %[output:0940899c]
plot(t*1000, err_ramp_lead, 'b', 'LineWidth', 2); %[output:0940899c]
plot(t*1000, err_ramp_lag, 'g', 'LineWidth', 2); %[output:0940899c]
grid on; %[output:0940899c]
xlabel('Tiempo (ms)'); %[output:0940899c]
ylabel('Error (rad)'); %[output:0940899c]
title('Error ante Rampa de Frecuencia'); %[output:0940899c]
legend('RC', 'Lead-Lag', 'Phase-Lag', 'Location', 'best'); %[output:0940899c]

%% Título general
sgtitle('Comparación de Errores de Fase en PLL', 'FontSize', 14, 'FontWeight', 'bold'); %[output:0940899c]

% Para la rampa, calculamos la pendiente (error crece linealmente)
idx_final = length(t)-100:length(t);  % Últimos 100 puntos
pend_RC = mean(diff(err_ramp_RC(idx_final))) / mean(diff(t(idx_final)));
pend_lead = mean(diff(err_ramp_lead(idx_final))) / mean(diff(t(idx_final)));
pend_lag = mean(diff(err_ramp_lag(idx_final))) / mean(diff(t(idx_final)));


%% Mostrar errores finales en consola
fprintf('\n=== ERRORES EN RÉGIMEN PERMANENTE (rad) ===\n\n'); %[output:670ec23b]
fprintf('                RC        Lead-Lag   Phase-Lag\n'); %[output:38f9f5af]
fprintf('Escalón fase: %.6f  %.6f  %.6f\n', ... %[output:group:6972e713] %[output:53c6084b]
    err_step_RC(end), err_step_lead(end), err_step_lag(end)); %[output:group:6972e713] %[output:53c6084b]
fprintf('Salto freq.:  %.5f  %.5f  %.6f\n', ... %[output:group:4c60fe47] %[output:3e88f9db]
    err_freq_RC(end), err_freq_lead(end), err_freq_lag(end)); %[output:group:4c60fe47] %[output:3e88f9db]
fprintf('Rampa freq.:  %.6f  %.6f  %.6f  (pendiente rad/s)\n', ... %[output:group:79892633] %[output:3921cc1a]
    pend_RC, pend_lead, pend_lag); %[output:group:79892633] %[output:3921cc1a]

disp(' '); %[output:9877248f]
disp('✓ Simulación completada'); %[output:694e875c]
%[output:694e875c]
%   data: {"dataType":"text","outputData":{"text":"✓ Simulación completada\n","truncated":false}}
%---
