clear; clc; close all;

p = params();
t = 0:p.dt:p.t_end;
N = numel(t);

% --- Target awal posisi 2D [mm] & waktu step [s] ---
init_x_mm = 50.0;
init_y_mm = 35.0;
t_step    = 0.050;

% Siapkan parameter khusus sumbu X & sumbu Y
p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

% --- Buat Figure & Komponen Plot ---
fig = figure('Name', '2D Point Position Control Simulation', 'Position', [80, 80, 1220, 700]);

% 1. Lintasan bidang X-Y (kiri)
ax1 = subplot(2, 2, [1, 3]);
ax1.Position = [0.06, 0.12, 0.30, 0.78];
grid(ax1, 'on');
axis(ax1, 'equal');
xlabel(ax1, 'Posisi X (mm)');
ylabel(ax1, 'Posisi Y (mm)');
title(ax1, 'Lintasan Planar 2D (POINT)');

% 2. Respon waktu posisi X dan Y (tengah atas)
ax2 = subplot(2, 2, 2);
ax2.Position = [0.42, 0.56, 0.27, 0.34];
grid(ax2, 'on');
xlabel(ax2, 'Waktu (s)');
ylabel(ax2, 'Posisi (mm)');
title(ax2, 'Respon Posisi X & Y (RK4)');

% 3. Kecepatan linier sumbu X dan Y (tengah bawah)
ax3 = subplot(2, 2, 4);
ax3.Position = [0.42, 0.12, 0.27, 0.34];
grid(ax3, 'on');
xlabel(ax3, 'Waktu (s)');
ylabel(ax3, 'Kecepatan (mm/s)');
title(ax3, 'Kecepatan Linier X & Y (RK4)');

% --- Panel Input & Informasi di Sisi Kanan Figure ---
panel = uipanel(fig, ...
    'Title', 'Target & Parameter PID', ...
    'FontWeight', 'bold', ...
    'Position', [0.72, 0.12, 0.26, 0.78]);

% Input X_ref [mm]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.06, 0.87, 0.44, 0.05], ...
    'String', 'X_ref (mm):', 'HorizontalAlignment', 'left');
edit_x = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.52, 0.87, 0.42, 0.055], ...
    'String', num2str(init_x_mm));

% Input Y_ref [mm]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.06, 0.80, 0.44, 0.05], ...
    'String', 'Y_ref (mm):', 'HorizontalAlignment', 'left');
edit_y = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.52, 0.80, 0.42, 0.055], ...
    'String', num2str(init_y_mm));

% Tombol jalankan ulang simulasi
btn_run = uicontrol(panel, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.06, 0.72, 0.88, 0.06], ...
    'String', 'Run Simulasi', 'FontWeight', 'bold');

% Kotak teks ringkasan error & gain PID di bawah tombol
info_box = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.06, 0.03, 0.88, 0.66], ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'FixedWidth', ...
    'FontSize', 8.5, ...
    'String', '');

% Hubungkan callback ke tombol dan Enter pada kolom input
sim_data = struct('p', p, 'p_x', p_x, 'p_y', p_y, 't', t, 'N', N, ...
    't_step', t_step, 'ax1', ax1, 'ax2', ax2, 'ax3', ax3, ...
    'edit_x', edit_x, 'edit_y', edit_y, 'info_box', info_box, ...
    'def_x_mm', init_x_mm, 'def_y_mm', init_y_mm);

btn_run.Callback = @(~, ~) update_simulation(sim_data);
edit_x.Callback  = @(~, ~) update_simulation(sim_data);
edit_y.Callback  = @(~, ~) update_simulation(sim_data);

% Jalankan pertama kali saat skrip dibuka
update_simulation(sim_data);

% --- Fungsi Callback & Simulasi ---
function update_simulation(d)
    val_x_mm = str2double(strtrim(d.edit_x.String));
    val_y_mm = str2double(strtrim(d.edit_y.String));

    if isnan(val_x_mm)
        val_x_mm = d.def_x_mm;
        d.edit_x.String = num2str(val_x_mm);
    end
    if isnan(val_y_mm)
        val_y_mm = d.def_y_mm;
        d.edit_y.String = num2str(val_y_mm);
    end

    x_ref = val_x_mm / 1000;
    y_ref = val_y_mm / 1000;

    x_ref_vec = zeros(1, d.N);
    y_ref_vec = zeros(1, d.N);
    idx_step = d.t >= d.t_step;
    x_ref_vec(idx_step) = x_ref;
    y_ref_vec(idx_step) = y_ref;

    [x_rk4, y_rk4, vx_rk4, vy_rk4, ~, ~] = run_2d_loop( ...
        d.p, d.p_x, d.p_y, d.t, x_ref_vec, y_ref_vec, 'rk4');
    [x_eu, y_eu, ~, ~, ~, ~] = run_2d_loop( ...
        d.p, d.p_x, d.p_y, d.t, x_ref_vec, y_ref_vec, 'euler');

    err_x_mm = (x_ref - x_rk4(end)) * 1000;
    err_y_mm = (y_ref - y_rk4(end)) * 1000;

    fprintf('Target (X, Y)    : (%.2f mm, %.2f mm)\n', val_x_mm, val_y_mm);
    fprintf('Posisi akhir RK4 : (%.2f mm, %.2f mm)\n', x_rk4(end)*1000, y_rk4(end)*1000);
    fprintf('Error akhir RK4  : (%.3f mm, %.3f mm)\n', err_x_mm, err_y_mm);

    % Update Plot 1: Lintasan X-Y
    cla(d.ax1);
    plot(d.ax1, x_ref_vec * 1000, y_ref_vec * 1000, '--k', 'LineWidth', 1.4); hold(d.ax1, 'on');
    plot(d.ax1, x_rk4 * 1000, y_rk4 * 1000, 'LineWidth', 1.8);
    plot(d.ax1, x_eu * 1000, y_eu * 1000, '-.', 'LineWidth', 1.3);
    plot(d.ax1, val_x_mm, val_y_mm, 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);
    grid(d.ax1, 'on');
    axis(d.ax1, 'equal');
    xlabel(d.ax1, 'Posisi X (mm)');
    ylabel(d.ax1, 'Posisi Y (mm)');
    title(d.ax1, 'Lintasan Planar 2D (POINT)');
    legend(d.ax1, 'Referensi', 'RK4', 'Forward Euler', 'Target', 'Location', 'best');

    % Update Plot 2: Respon Posisi X & Y
    cla(d.ax2);
    plot(d.ax2, d.t, x_ref_vec * 1000, '--', 'LineWidth', 1.2); hold(d.ax2, 'on');
    plot(d.ax2, d.t, x_rk4 * 1000, 'LineWidth', 1.6);
    plot(d.ax2, d.t, y_ref_vec * 1000, '--', 'LineWidth', 1.2);
    plot(d.ax2, d.t, y_rk4 * 1000, 'LineWidth', 1.6);
    grid(d.ax2, 'on');
    xlabel(d.ax2, 'Waktu (s)');
    ylabel(d.ax2, 'Posisi (mm)');
    title(d.ax2, 'Respon Posisi X & Y (RK4)');
    legend(d.ax2, 'X ref', 'X RK4', 'Y ref', 'Y RK4', 'Location', 'best');

    % Update Plot 3: Kecepatan Linier X & Y
    cla(d.ax3);
    plot(d.ax3, d.t, vx_rk4 * 1000, 'LineWidth', 1.4); hold(d.ax3, 'on');
    plot(d.ax3, d.t, vy_rk4 * 1000, 'LineWidth', 1.4);
    grid(d.ax3, 'on');
    xlabel(d.ax3, 'Waktu (s)');
    ylabel(d.ax3, 'Kecepatan (mm/s)');
    title(d.ax3, 'Kecepatan Linier X & Y (RK4)');
    legend(d.ax3, 'v_x', 'v_y', 'Location', 'best');

    % Update teks ringkasan di panel kanan
    info_lines = {
        sprintf('t_step  : %.3f s', d.t_step)
        sprintf('Err RK4 : (%.2f, %.2f) mm', err_x_mm, err_y_mm)
        ''
        '--- PID POS X ---'
        sprintf('Kp_x : %.3g', d.p.pid_pos.x.Kp)
        sprintf('Ki_x : %.3g', d.p.pid_pos.x.Ki)
        sprintf('Kd_x : %.3g', d.p.pid_pos.x.Kd)
        sprintf('N_x  : %.3g', d.p.pid_pos.x.N)
        ''
        '--- PID POS Y ---'
        sprintf('Kp_y : %.3g', d.p.pid_pos.y.Kp)
        sprintf('Ki_y : %.3g', d.p.pid_pos.y.Ki)
        sprintf('Kd_y : %.3g', d.p.pid_pos.y.Kd)
        sprintf('N_y  : %.3g', d.p.pid_pos.y.N)
        ''
        sprintf('m_x=%.3f m_y=%.3f', d.p.m_x, d.p.m_y)
        sprintf('dt=%.4g t_end=%.2g', d.p.dt, d.p.t_end)
    };
    d.info_box.String = info_lines;
end

function [x_hist, y_hist, vx_hist, vy_hist, ux_hist, uy_hist] = run_2d_loop( ...
        p, p_x, p_y, t, x_ref_vec, y_ref_vec, method)
    N = numel(t);

    state_x = [0.0; 0.0; 0.0];
    state_y = [0.0; 0.0; 0.0];

    pid_state_x = [];
    pid_state_y = [];

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

        err_x = x_ref_vec(k) - pos_x;
        err_y = y_ref_vec(k) - pos_y;

        [ux, pid_state_x] = pid_controller( ...
            err_x, p_x.dt, p.pid_pos.x, pid_state_x, p_x.u_min, p_x.u_max);
        [uy, pid_state_y] = pid_controller( ...
            err_y, p_y.dt, p.pid_pos.y, pid_state_y, p_y.u_min, p_y.u_max);

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