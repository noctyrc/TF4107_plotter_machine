clear; clc; close all;

p = params();
t = 0:p.dt:p.t_end;
N = numel(t);

% initial target
init_x_mm = 50.0;
init_y_mm = 35.0;
t_step    = 0.050;

% initial x
init_qx1 = 5e6;
init_qx2 = 50.0;
init_qx3 = 0.1;
init_qx4 = 1e8;
init_rx  = 0.05;

% initial y
init_qy1 = 5e6;
init_qy2 = 50.0;
init_qy3 = 0.1;
init_qy4 = 1e8;
init_ry  = 0.05;

% parameter sumbu X Y
p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

fig = figure('Name', '2D LQI Position Control (Per-Axis Tuning)', 'Position', [60, 50, 1340, 760]);

% lintasan bidang XY
ax1 = subplot(2, 2, [1, 3]);
ax1.Position = [0.05, 0.11, 0.27, 0.80];
grid(ax1, 'on');
axis(ax1, 'equal');
xlabel(ax1, 'Posisi X (mm)');
ylabel(ax1, 'Posisi Y (mm)');
title(ax1, 'Lintasan Planar 2D - LQI');

% respon waktu posisi XY
ax2 = subplot(2, 2, 2);
ax2.Position = [0.37, 0.56, 0.26, 0.35];
grid(ax2, 'on');
xlabel(ax2, 'Waktu (s)');
ylabel(ax2, 'Posisi (mm)');
title(ax2, 'Respon Posisi X & Y (RK4)');

% sinyal kontrol tegangan u_x dan u_y
ax3 = subplot(2, 2, 4);
ax3.Position = [0.37, 0.11, 0.26, 0.35];
grid(ax3, 'on');
xlabel(ax3, 'Waktu (s)');
ylabel(ax3, 'Tegangan u (V)');
title(ax3, 'Sinyal Kontrol Tegangan LQI (RK4)');

% panel kontrol & tuning
panel = uipanel(fig, ...
    'Title', 'Target & Tuning LQI (X & Y)', ...
    'FontWeight', 'bold', ...
    'Position', [0.66, 0.11, 0.32, 0.80]);

% target posisi xy
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04, 0.92, 0.22, 0.045], ...
    'String', 'X_ref (mm):', 'HorizontalAlignment', 'left');
edit_x = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.26, 0.92, 0.21, 0.05], ...
    'String', num2str(init_x_mm));

uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.51, 0.92, 0.22, 0.045], ...
    'String', 'Y_ref (mm):', 'HorizontalAlignment', 'left');
edit_y = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.73, 0.92, 0.22, 0.05], ...
    'String', num2str(init_y_mm));

% header
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.26, 0.86, 0.32, 0.04], ...
    'String', 'SUMBU X', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.62, 0.86, 0.33, 0.04], ...
    'String', 'SUMBU Y', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Q1 (pos)
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04, 0.80, 0.20, 0.045], ...
    'String', 'Q1 (pos):', 'HorizontalAlignment', 'left');
edit_qx1 = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.26, 0.80, 0.32, 0.05], ...
    'String', num2str(init_qx1));
edit_qy1 = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.62, 0.80, 0.33, 0.05], ...
    'String', num2str(init_qy1));

% Q2 (vel)
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04, 0.74, 0.20, 0.045], ...
    'String', 'Q2 (vel):', 'HorizontalAlignment', 'left');
edit_qx2 = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.26, 0.74, 0.32, 0.05], ...
    'String', num2str(init_qx2));
edit_qy2 = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.62, 0.74, 0.33, 0.05], ...
    'String', num2str(init_qy2));

% Q4 (int)
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04, 0.68, 0.20, 0.045], ...
    'String', 'Q4 (int):', 'HorizontalAlignment', 'left');
edit_qx4 = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.26, 0.68, 0.32, 0.05], ...
    'String', num2str(init_qx4));
edit_qy4 = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.62, 0.68, 0.33, 0.05], ...
    'String', num2str(init_qy4));

% R (input)
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04, 0.62, 0.20, 0.045], ...
    'String', 'R (input):', 'HorizontalAlignment', 'left');
edit_rx = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.26, 0.62, 0.32, 0.05], ...
    'String', num2str(init_rx));
edit_ry = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.62, 0.62, 0.33, 0.05], ...
    'String', num2str(init_ry));

% button run
btn_run = uicontrol(panel, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.04, 0.54, 0.91, 0.06], ...
    'String', 'Run Simulasi', 'FontWeight', 'bold');

% teks summary
info_box = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.04, 0.02, 0.91, 0.50], ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'FixedWidth', ...
    'FontSize', 8.0, ...
    'String', '');

sim_data = struct('p', p, 'p_x', p_x, 'p_y', p_y, ...
    't', t, 'N', N, 't_step', t_step, ...
    'ax1', ax1, 'ax2', ax2, 'ax3', ax3, ...
    'edit_x', edit_x, 'edit_y', edit_y, ...
    'edit_qx1', edit_qx1, 'edit_qx2', edit_qx2, 'edit_qx4', edit_qx4, 'edit_rx', edit_rx, ...
    'edit_qy1', edit_qy1, 'edit_qy2', edit_qy2, 'edit_qy4', edit_qy4, 'edit_ry', edit_ry, ...
    'info_box', info_box, ...
    'def_x_mm', init_x_mm, 'def_y_mm', init_y_mm, ...
    'def_qx1', init_qx1, 'def_qx2', init_qx2, 'def_qx3', init_qx3, 'def_qx4', init_qx4, 'def_rx', init_rx, ...
    'def_qy1', init_qy1, 'def_qy2', init_qy2, 'def_qy3', init_qy3, 'def_qy4', init_qy4, 'def_ry', init_ry);

cb = @(~, ~) update_lqi_simulation(sim_data);
btn_run.Callback  = cb;
edit_x.Callback   = cb;
edit_y.Callback   = cb;
edit_qx1.Callback = cb;
edit_qx2.Callback = cb;
edit_qx4.Callback = cb;
edit_rx.Callback  = cb;
edit_qy1.Callback = cb;
edit_qy2.Callback = cb;
edit_qy4.Callback = cb;
edit_ry.Callback  = cb;

update_lqi_simulation(sim_data);

% fungsi callback & simulasi
function update_lqi_simulation(d)
    val_x_mm = parse_pos_input(d.edit_x, d.def_x_mm);
    val_y_mm = parse_pos_input(d.edit_y, d.def_y_mm);

    qx1 = parse_pos_input(d.edit_qx1, d.def_qx1);
    qx2 = parse_pos_input(d.edit_qx2, d.def_qx2);
    qx4 = parse_pos_input(d.edit_qx4, d.def_qx4);
    rx  = parse_pos_input(d.edit_rx,  d.def_rx);

    qy1 = parse_pos_input(d.edit_qy1, d.def_qy1);
    qy2 = parse_pos_input(d.edit_qy2, d.def_qy2);
    qy4 = parse_pos_input(d.edit_qy4, d.def_qy4);
    ry  = parse_pos_input(d.edit_ry,  d.def_ry);

    Qx = diag([qx1, qx2, d.def_qx3, qx4]);
    Qy = diag([qy1, qy2, d.def_qy3, qy4]);

    Kx = lqr_gain_axis(d.p_x, Qx, rx);
    Ky = lqr_gain_axis(d.p_y, Qy, ry);

    x_ref_vec = zeros(1, d.N);
    y_ref_vec = zeros(1, d.N);
    idx_step  = d.t >= d.t_step;
    x_ref_vec(idx_step) = val_x_mm / 1000;
    y_ref_vec(idx_step) = val_y_mm / 1000;

    [x_rk4, y_rk4, ~, ~, ux_rk4, uy_rk4] = run_lqi_loop( ...
        d.p_x, d.p_y, Kx, Ky, d.t, x_ref_vec, y_ref_vec, 'rk4');
    [x_eu, y_eu, ~, ~, ~, ~] = run_lqi_loop( ...
        d.p_x, d.p_y, Kx, Ky, d.t, x_ref_vec, y_ref_vec, 'euler');

    err_x_mm = val_x_mm - x_rk4(end) * 1000;
    err_y_mm = val_y_mm - y_rk4(end) * 1000;
    max_u_x  = max(abs(ux_rk4));
    max_u_y  = max(abs(uy_rk4));

    fprintf(['X: Q=[%.1e, %.3g, %.2g, %.1e], R=%.4g | ', ...
             'Y: Q=[%.1e, %.3g, %.2g, %.1e], R=%.4g | ', ...
             'Err RK4: (%.3f, %.3f) mm\n'], ...
        qx1, qx2, d.def_qx3, qx4, rx, ...
        qy1, qy2, d.def_qy3, qy4, ry, ...
        err_x_mm, err_y_mm);

    cla(d.ax1);
    plot(d.ax1, x_ref_vec*1000, y_ref_vec*1000, '--k', 'LineWidth', 1.4); hold(d.ax1, 'on');
    plot(d.ax1, x_rk4*1000, y_rk4*1000, 'LineWidth', 1.8);
    plot(d.ax1, x_eu*1000, y_eu*1000, '-.', 'LineWidth', 1.3);
    plot(d.ax1, val_x_mm, val_y_mm, 'ro', 'MarkerSize', 7, 'LineWidth', 1.5);
    grid(d.ax1, 'on');
    axis(d.ax1, 'equal');
    xlabel(d.ax1, 'Posisi X (mm)');
    ylabel(d.ax1, 'Posisi Y (mm)');
    title(d.ax1, 'Lintasan Planar 2D - LQI');
    legend(d.ax1, 'Referensi', 'RK4', 'Forward Euler', 'Target', 'Location', 'best');

    cla(d.ax2);
    plot(d.ax2, d.t, x_ref_vec*1000, '--', 'LineWidth', 1.2); hold(d.ax2, 'on');
    plot(d.ax2, d.t, x_rk4*1000, 'LineWidth', 1.6);
    plot(d.ax2, d.t, y_ref_vec*1000, '--', 'LineWidth', 1.2);
    plot(d.ax2, d.t, y_rk4*1000, 'LineWidth', 1.6);
    grid(d.ax2, 'on');
    xlabel(d.ax2, 'Waktu (s)');
    ylabel(d.ax2, 'Posisi (mm)');
    title(d.ax2, 'Respon Posisi X & Y (RK4)');
    legend(d.ax2, 'X ref', 'X RK4', 'Y ref', 'Y RK4', 'Location', 'best');

    cla(d.ax3);
    plot(d.ax3, d.t, ux_rk4, 'LineWidth', 1.4); hold(d.ax3, 'on');
    plot(d.ax3, d.t, uy_rk4, 'LineWidth', 1.4);
    grid(d.ax3, 'on');
    xlabel(d.ax3, 'Waktu (s)');
    ylabel(d.ax3, 'Tegangan u (V)');
    title(d.ax3, 'Sinyal Kontrol Tegangan LQI (RK4)');
    legend(d.ax3, 'u_x', 'u_y', 'Location', 'best');

    % update teks summary
    info_lines = {
        sprintf('Err RK4 : (%.2f, %.2f) mm', err_x_mm, err_y_mm)
        sprintf('|u| max : (%.1f, %.1f) V', max_u_x, max_u_y)
        sprintf('Rx=%.3g | Ry=%.3g', rx, ry)
        ''
        sprintf('Kx: [%6.1f %6.2f %6.2f %6.0f]', Kx(1), Kx(2), Kx(3), Kx(4))
        sprintf('Ky: [%6.1f %6.2f %6.2f %6.0f]', Ky(1), Ky(2), Ky(3), Ky(4))
    };
    d.info_box.String = info_lines;
    drawnow;
end

function val = parse_pos_input(edit_ctrl, default_val)
    val = str2double(strtrim(edit_ctrl.String));
    if isnan(val) || ~isfinite(val) || val <= 0
        val = default_val;
        edit_ctrl.String = num2str(val);
    end
end

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