clear; clc; close all;

p = params();
t = 0:p.dt:p.t_end;
N = numel(t);

% --- Target posisi 2D [mm] & waktu step [s] ---
x_ref_mm = 50.0;
y_ref_mm = 35.0;
t_step   = 0.050;

% Bobot LQI terpisah per sumbu (bisa juga dipindahkan ke params.m)
Qx = diag([5e6, 50, 0.1, 1e8]);
Rx = 0.05;
Qy = diag([5e6, 50, 0.1, 1e8]);
Ry = 0.05;

% parameter sumbu X Y
p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

% gain LQI & eigenvalue closed loop per sumbu
Kx = lqr_gain_axis(p_x, Qx, Rx);
Ky = lqr_gain_axis(p_y, Qy, Ry);

% referensi step posisi [m]
x_ref_vec = zeros(1, N);
y_ref_vec = zeros(1, N);
idx_step = t >= t_step;
x_ref_vec(idx_step) = x_ref_mm / 1000;
y_ref_vec(idx_step) = y_ref_mm / 1000;

% simulasi RK4 & forward euler
[x_rk4, y_rk4, vx_rk4, vy_rk4, ux_rk4, uy_rk4] = run_lqi_loop( ...
    p_x, p_y, Kx, Ky, t, x_ref_vec, y_ref_vec, 'rk4');
[x_eu, y_eu, ~, ~, ~, ~] = run_lqi_loop( ...
    p_x, p_y, Kx, Ky, t, x_ref_vec, y_ref_vec, 'euler');

err_x_mm = x_ref_mm - x_rk4(end) * 1000;
err_y_mm = y_ref_mm - y_rk4(end) * 1000;

fprintf('Target (X, Y)    : (%.2f mm, %.2f mm)\n', x_ref_mm, y_ref_mm);
fprintf('Posisi akhir RK4 : (%.2f mm, %.2f mm)\n', x_rk4(end)*1000, y_rk4(end)*1000);
fprintf('Error akhir RK4  : (%.3f mm, %.3f mm)\n', err_x_mm, err_y_mm);

% visualisasi
figure('Name', '2D LQI Position Control Simulation', 'Position', [90, 90, 1140, 650]);

subplot(2, 2, [1, 3]);
plot(x_ref_vec*1000, y_ref_vec*1000, '--k', 'LineWidth', 1.4); hold on;
plot(x_rk4*1000, y_rk4*1000, 'LineWidth', 1.8);
plot(x_eu*1000, y_eu*1000, '-.', 'LineWidth', 1.3);
plot(x_ref_mm, y_ref_mm, 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);
grid on; axis equal;
xlabel('Posisi X (mm)'); ylabel('Posisi Y (mm)');
title('Lintasan Planar 2D - LQI');
legend('Referensi', 'RK4', 'Forward Euler', 'Target', 'Location', 'best');

subplot(2, 2, 2);
plot(t, x_ref_vec*1000, '--', 'LineWidth', 1.2); hold on;
plot(t, x_rk4*1000, 'LineWidth', 1.6);
plot(t, y_ref_vec*1000, '--', 'LineWidth', 1.2);
plot(t, y_rk4*1000, 'LineWidth', 1.6);
grid on;
xlabel('Waktu (s)'); ylabel('Posisi (mm)');
title('Respon Posisi X & Y (RK4)');
legend('X ref', 'X RK4', 'Y ref', 'Y RK4', 'Location', 'best');

subplot(2, 2, 4);
plot(t, ux_rk4, 'LineWidth', 1.4); hold on;
plot(t, uy_rk4, 'LineWidth', 1.4);
grid on;
xlabel('Waktu (s)'); ylabel('Tegangan u (V)');
title('Sinyal Kontrol Tegangan LQI (RK4)');
legend('u_x', 'u_y', 'Location', 'best');

function [x_hist, y_hist, vx_hist, vy_hist, ux_hist, uy_hist] = run_lqi_loop( ...
        p_x, p_y, Kx, Ky, t, x_ref_vec, y_ref_vec, method)
    N = numel(t);
    state_x = [0.0; 0.0; 0.0];
    state_y = [0.0; 0.0; 0.0];
    qx = 0.0;
    qy = 0.0;

    x_hist  = zeros(1, N);
    y_hist  = zeros(1, N);
    vx_hist = zeros(1, N);
    vy_hist = zeros(1, N);
    ux_hist = zeros(1, N);
    uy_hist = zeros(1, N);

    ode_x = @(tk, xk, uk) dynamics_model(tk, xk, uk, p_x);
    ode_y = @(tk, yk, uk) dynamics_model(tk, yk, uk, p_y);

    for k = 1:N
        pos_x = p_x.rp * state_x(1);
        vel_x = p_x.rp * state_x(2);
        pos_y = p_y.rp * state_y(1);
        vel_y = p_y.rp * state_y(2);

        x_hist(k)  = pos_x;
        y_hist(k)  = pos_y;
        vx_hist(k) = vel_x;
        vy_hist(k) = vel_y;

        [ux, qx] = lqi_controller(x_ref_vec(k), 0.0, state_x, p_x.rp, Kx, qx, p_x.dt, p_x.u_min, p_x.u_max);
        [uy, qy] = lqi_controller(y_ref_vec(k), 0.0, state_y, p_y.rp, Ky, qy, p_y.dt, p_y.u_min, p_y.u_max);

        ux_hist(k) = ux;
        uy_hist(k) = uy;

        if k < N
            switch lower(method)
                case 'rk4'
                    state_x = step_rk4(ode_x, t(k), state_x, ux, p_x.dt);
                    state_y = step_rk4(ode_y, t(k), state_y, uy, p_y.dt);
                case 'euler'
                    state_x = step_euler(ode_x, t(k), state_x, ux, p_x.dt);
                    state_y = step_euler(ode_y, t(k), state_y, uy, p_y.dt);
                otherwise
                    error('Metode integrasi tidak dikenal: %s', method);
            end
        end
    end
end