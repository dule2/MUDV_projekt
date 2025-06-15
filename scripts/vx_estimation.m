% Već postojeće vrijednosti
rmse_imu = sqrt(mean((vx_est - vx_true).^2));
fprintf("RMSE (IMU): %.4f m/s\n", rmse_imu);

% Nova vrijednost: estimacija pomoću kotača
rmse_wheels = sqrt(mean((vx_wheels - vx_true).^2));
fprintf("RMSE (kotači): %.4f m/s\n", rmse_wheels);

% Usporedni graf
plot(vx_true, 'k', 'LineWidth', 1.5); hold on;
plot(vx_est, 'r--', 'LineWidth', 1.5);
plot(vx_wheels, 'b:', 'LineWidth', 1.5);
legend('Stvarna brzina', 'Est. IMU', 'Est. kotači');
xlabel('Uzorci'); ylabel('Brzina [m/s]');
title('Usporedba različitih estimacija uzdužne brzine');
grid on;

alpha = 0.7;  % Povjerenje u kotače (npr. 70%)
vx_combined = alpha * vx_wheels + (1 - alpha) * vx_est;

rmse_combined = sqrt(mean((vx_combined - vx_true).^2));
fprintf("RMSE (kombinirano): %.4f m/s\n", rmse_combined);

% Prikaz na grafu
plot(vx_combined, 'g-.', 'LineWidth', 1.5);
legend('Stvarna', 'IMU', 'Kotači', 'Kombinirana');

%------------------KALMAN----------------------------------------
% Parametri Kalman filtra
dt = tout(2) - tout(1);   % razmak između uzoraka
Q = 0.1;    % varijanca modela (IMU)
R = 0.5;    % varijanca mjerenja (kotači)

% Inicijalizacija
N = length(tout);
x_est = 0;   % početna brzina
P = 1;       % početna kovarijanca

vx_kalman = zeros(N,1);

% Kalman filter
for k = 1:N
    % 1. Predikcija
    x_pred = x_est + a_x(k) * dt;
    P_pred = P + Q;

    % 2. Korekcija
    K = P_pred / (P_pred + R);                     % Kalman gain
    x_est = x_pred + K * (vx_wheels(k) - x_pred);  % Ažurirana brzina
    P = (1 - K) * P_pred;                          % Ažurirana kovarijanca

    % Spremi rezultat
    vx_kalman(k) = x_est;
end

% RMSE analiza
rmse_kalman = sqrt(mean((vx_kalman - vx_true).^2));
fprintf("RMSE (Kalman): %.4f m/s\n", rmse_kalman);

% Grafička usporedba
figure;
plot(tout, vx_true, 'k', 'LineWidth', 1.5); hold on;
plot(tout, vx_wheels, 'b--', 'LineWidth', 1.2);
plot(tout, vx_kalman, 'g-.', 'LineWidth', 1.2);
legend('Stvarna brzina', 'Kotači', 'Kalman filter');
xlabel('Vrijeme [s]');
ylabel('Brzina [m/s]');
title('Usporedba uzdužnih brzina');
grid on;