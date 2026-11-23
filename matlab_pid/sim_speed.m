clear; clc; close all;

p = params();
t = 0:p.dt:p.t_end;
N = numel(t);

% target awal kecepatan [mm/s]
init_vx_mm_s = 120.0; % mm/s (0.120 m/s)
init_vy_mm_s = 80.0;  % mm/s (0.080 m/s)

% parameter sumbu X & sumbu Y
p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

fig = figure('Name', '2-Axis Speed Control Simulation', 'Position', [90, 90, 1120, 650]);

% grafik kecepatan linier X Y
ax1 = subplot(2, 1, 1);
ax1.Position = [0.08, 0.56, 0.60, 0.35];
grid(ax1, 'on');
ylabel(ax1, 'Kecepatan (mm/s)');
title(ax1, 'Kontrol Kecepatan Sumbu X & Y Plotter (RK4)');

% grafik tegangan input u_x u_y
ax2 = subplot(2, 1, 2);
ax2.Position = [0.08, 0.11, 0.60, 0.35];
grid(ax2, 'on');
xlabel(ax2, 'Waktu (s)');
ylabel(ax2, 'Tegangan Input u (V)');

% panel input dan informasi samping
panel = uipanel(fig, ...
    'Title', 'Target & Parameter PID', ...
    'FontWeight', 'bold', ...
    'Position', [0.71, 0.11, 0.26, 0.80]);

% input vx_ref [mm/s]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.06, 0.87, 0.46, 0.05], ...
    'String', 'vx_ref (mm/s):', 'HorizontalAlignment', 'left');
edit_vx = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.54, 0.87, 0.40, 0.055], ...
    'String', num2str(init_vx_mm_s));

% input vy_ref [mm/s]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.06, 0.80, 0.46, 0.05], ...
    'String', 'vy_ref (mm/s):', 'HorizontalAlignment', 'left');
edit_vy = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.54, 0.80, 0.40, 0.055], ...
    'String', num2str(init_vy_mm_s));

% button run
btn_run = uicontrol(panel, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.06, 0.72, 0.88, 0.06], ...
    'String', 'Run Simulasi', 'FontWeight', 'bold');

% kotak teks ringkasan hasil & gain PID di bawah tombol
info_box = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.06, 0.03, 0.88, 0.66], ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'FixedWidth', ...
    'FontSize', 8.5, ...
    'String', '');

sim_data = struct('p', p, 'p_x', p_x, 'p_y', p_y, 't', t, 'N', N, ...
    'ax1', ax1, 'ax2', ax2, 'edit_vx', edit_vx, 'edit_vy', edit_vy, ...
    'info_box', info_box, 'def_vx_mm_s', init_vx_mm_s, 'def_vy_mm_s', init_vy_mm_s);

btn_run.Callback = @(~, ~) update_speed_simulation(sim_data);
edit_vx.Callback = @(~, ~) update_speed_simulation(sim_data);
edit_vy.Callback = @(~, ~) update_speed_simulation(sim_data);

update_speed_simulation(sim_data);

function update_speed_simulation(d)
    val_vx_mm_s = str2double(strtrim(d.edit_vx.String));
    val_vy_mm_s = str2double(strtrim(d.edit_vy.String));

    if isnan(val_vx_mm_s)
        val_vx_mm_s = d.def_vx_mm_s;
        d.edit_vx.String = num2str(val_vx_mm_s);
    end
    if isnan(val_vy_mm_s)
        val_vy_mm_s = d.def_vy_mm_s;
        d.edit_vy.String = num2str(val_vy_mm_s);
    end

    vx_ref = val_vx_mm_s / 1000; 
    vy_ref = val_vy_mm_s / 1000; 

    state_x = [0.0; 0.0; 0.0];
    state_y = [0.0; 0.0; 0.0];

    pid_state_x = [];
    pid_state_y = [];

    vx_hist = zeros(1, d.N);
    vy_hist = zeros(1, d.N);
    ux_hist = zeros(1, d.N);
    uy_hist = zeros(1, d.N);

    ode_x = @(tk, xk, uk) dynamics_model(tk, xk, uk, d.p_x);
    ode_y = @(tk, yk, uk) dynamics_model(tk, yk, uk, d.p_y);

    for k = 1:d.N
        vel_x = d.p_x.rp * state_x(2);
        vel_y = d.p_y.rp * state_y(2);

        vx_hist(k) = vel_x;
        vy_hist(k) = vel_y;

        err_x = vx_ref - vel_x;
        err_y = vy_ref - vel_y;

        [ux, pid_state_x] = pid_controller( ...
            err_x, d.p_x.dt, d.p.pid_speed.x, pid_state_x, d.p_x.u_min, d.p_x.u_max);
        [uy, pid_state_y] = pid_controller( ...
            err_y, d.p_y.dt, d.p.pid_speed.y, pid_state_y, d.p_y.u_min, d.p_y.u_max);

        ux_hist(k) = ux;
        uy_hist(k) = uy;

        if k < d.N
            state_x = step_rk4(ode_x, d.t(k), state_x, ux, d.p_x.dt);
            state_y = step_rk4(ode_y, d.t(k), state_y, uy, d.p_y.dt);
        end
    end

    fprintf('Kecepatan akhir X: %.2f mm/s (Target: %.2f mm/s)\n', vx_hist(end)*1000, val_vx_mm_s);
    fprintf('Kecepatan akhir Y: %.2f mm/s (Target: %.2f mm/s)\n', vy_hist(end)*1000, val_vy_mm_s);

    cla(d.ax1);
    plot(d.ax1, d.t, vx_hist * 1000, 'LineWidth', 1.6); hold(d.ax1, 'on');
    plot(d.ax1, d.t, vy_hist * 1000, 'LineWidth', 1.6);
    yline(d.ax1, val_vx_mm_s, '--', sprintf('v_x ref (%.0f mm/s)', val_vx_mm_s), 'LineWidth', 1.1);
    yline(d.ax1, val_vy_mm_s, ':',  sprintf('v_y ref (%.0f mm/s)', val_vy_mm_s), 'LineWidth', 1.1);
    grid(d.ax1, 'on');
    ylabel(d.ax1, 'Kecepatan (mm/s)');
    title(d.ax1, 'Kontrol Kecepatan Sumbu X & Y Plotter (RK4)');
    legend(d.ax1, 'v_x aktual', 'v_y aktual', 'Location', 'southeast');

    cla(d.ax2);
    plot(d.ax2, d.t, ux_hist, 'LineWidth', 1.4); hold(d.ax2, 'on');
    plot(d.ax2, d.t, uy_hist, 'LineWidth', 1.4);
    grid(d.ax2, 'on');
    xlabel(d.ax2, 'Waktu (s)');
    ylabel(d.ax2, 'Tegangan Input u (V)');
    legend(d.ax2, 'u_x', 'u_y', 'Location', 'southeast');

    % teks info
    info_lines = {
        sprintf('vx_end : %.1f mm/s', vx_hist(end)*1000)
        sprintf('vy_end : %.1f mm/s', vy_hist(end)*1000)
        ''
        'PID SPEED X'
        sprintf('Kp_x : %.3g', d.p.pid_speed.x.Kp)
        sprintf('Ki_x : %.3g', d.p.pid_speed.x.Ki)
        sprintf('Kd_x : %.3g', d.p.pid_speed.x.Kd)
        sprintf('N_x  : %.3g', d.p.pid_speed.x.N)
        ''
        'PID SPEED Y'
        sprintf('Kp_y : %.3g', d.p.pid_speed.y.Kp)
        sprintf('Ki_y : %.3g', d.p.pid_speed.y.Ki)
        sprintf('Kd_y : %.3g', d.p.pid_speed.y.Kd)
        sprintf('N_y  : %.3g', d.p.pid_speed.y.N)
        ''
        sprintf('m_x=%.3f m_y=%.3f', d.p.m_x, d.p.m_y)
        sprintf('dt=%.4g t_end=%.2g', d.p.dt, d.p.t_end)
    };
    d.info_box.String = info_lines;
end