clear; clc; close all;

p = params();
t = 0:p.dt:p.t_end;
N = numel(t);

% target awal posisi 2D [mm] & waktu step [s]
init_x_mm = 50.0;
init_y_mm = 35.0;
t_step    = 0.050;

% nilai awal PID diambil dari params()
def_pid_x = p.pid_pos.x;
def_pid_y = p.pid_pos.y;

% parameter sumbu X & sumbu Y
p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

fig = figure('Name', '2D Point Position Control Simulation', 'Position', [80, 80, 1240, 720]);

% lintasan bidang X Y
ax1 = subplot(2, 2, [1, 3]);
ax1.Position = [0.06, 0.12, 0.30, 0.78];
grid(ax1, 'on');
axis(ax1, 'equal');
xlabel(ax1, 'Posisi X (mm)');
ylabel(ax1, 'Posisi Y (mm)');
title(ax1, 'Lintasan Planar 2D (POINT)');

% respon waktu posisi X Y
ax2 = subplot(2, 2, 2);
ax2.Position = [0.41, 0.56, 0.27, 0.34];
grid(ax2, 'on');
xlabel(ax2, 'Waktu (s)');
ylabel(ax2, 'Posisi (mm)');
title(ax2, 'Respon Posisi X & Y (RK4)');

% kecepatan linier sumbu X Y
ax3 = subplot(2, 2, 4);
ax3.Position = [0.41, 0.12, 0.27, 0.34];
grid(ax3, 'on');
xlabel(ax3, 'Waktu (s)');
ylabel(ax3, 'Kecepatan (mm/s)');
title(ax3, 'Kecepatan Linier X & Y (RK4)');

% panel input & informasi
panel = uipanel(fig, ...
    'Title', 'Target & Parameter PID', ...
    'FontWeight', 'bold', ...
    'Position', [0.71, 0.08, 0.27, 0.84]);

% Input X_ref & Y_ref [mm]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.90, 0.42, 0.045], ...
    'String', 'X_ref (mm):', 'HorizontalAlignment', 'left');
edit_x = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.50, 0.90, 0.45, 0.05], ...
    'String', num2str(init_x_mm));

uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.84, 0.42, 0.045], ...
    'String', 'Y_ref (mm):', 'HorizontalAlignment', 'left');
edit_y = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.50, 0.84, 0.45, 0.05], ...
    'String', num2str(init_y_mm));

% Header kolom PID Sumbu X & Sumbu Y
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.78, 0.36, 0.04], ...
    'String', 'Gain PID', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.44, 0.78, 0.24, 0.04], ...
    'String', 'Sumbu X', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.71, 0.78, 0.24, 0.04], ...
    'String', 'Sumbu Y', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% Helper membuat baris edit PID X dan Y
make_pid_row = @(label_str, y_pos, val_x, val_y) deal( ...
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', 'Position', [0.05, y_pos, 0.36, 0.045], ...
        'String', label_str, 'HorizontalAlignment', 'left'), ...
    uicontrol(panel, 'Style', 'edit', ...
        'Units', 'normalized', 'Position', [0.44, y_pos, 0.24, 0.048], ...
        'String', num2str(val_x)), ...
    uicontrol(panel, 'Style', 'edit', ...
        'Units', 'normalized', 'Position', [0.71, y_pos, 0.24, 0.048], ...
        'String', num2str(val_y)) ...
);

[~, edit_kpx, edit_kpy] = make_pid_row('Kp:', 0.725, def_pid_x.Kp, def_pid_y.Kp);
[~, edit_kix, edit_kiy] = make_pid_row('Ki:', 0.670, def_pid_x.Ki, def_pid_y.Ki);
[~, edit_kdx, edit_kdy] = make_pid_row('Kd:', 0.615, def_pid_x.Kd, def_pid_y.Kd);
[~, edit_nx,  edit_ny]  = make_pid_row('N :', 0.560, def_pid_x.N,  def_pid_y.N);

% button run
btn_run = uicontrol(panel, 'Style', 'pushbutton', ...
    'Units', 'normalized', 'Position', [0.05, 0.485, 0.90, 0.058], ...
    'String', 'Run Simulasi', 'FontWeight', 'bold');

% teks ringkasan error & parameter di bawah tombol
info_box = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.02, 0.90, 0.44], ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'FixedWidth', ...
    'FontSize', 8.5, ...
    'String', '');

sim_data = struct('p', p, 'p_x', p_x, 'p_y', p_y, 't', t, 'N', N, ...
    't_step', t_step, 'ax1', ax1, 'ax2', ax2, 'ax3', ax3, ...
    'edit_x', edit_x, 'edit_y', edit_y, ...
    'edit_kpx', edit_kpx, 'edit_kix', edit_kix, 'edit_kdx', edit_kdx, 'edit_nx', edit_nx, ...
    'edit_kpy', edit_kpy, 'edit_kiy', edit_kiy, 'edit_kdy', edit_kdy, 'edit_ny', edit_ny, ...
    'def_pid_x', def_pid_x, 'def_pid_y', def_pid_y, ...
    'info_box', info_box, 'def_x_mm', init_x_mm, 'def_y_mm', init_y_mm);

all_edits = [edit_x, edit_y, edit_kpx, edit_kix, edit_kdx, edit_nx, ...
             edit_kpy, edit_kiy, edit_kdy, edit_ny];
btn_run.Callback = @(~, ~) update_simulation(sim_data);
for h = all_edits
    h.Callback = @(~, ~) update_simulation(sim_data);
end

update_simulation(sim_data);

% fungsi callback & simulasi ---
function update_simulation(d)
    val_x_mm = parse_edit_or_default(d.edit_x, d.def_x_mm);
    val_y_mm = parse_edit_or_default(d.edit_y, d.def_y_mm);

    pid_x = d.def_pid_x;
    pid_x.Kp = parse_edit_or_default(d.edit_kpx, d.def_pid_x.Kp);
    pid_x.Ki = parse_edit_or_default(d.edit_kix, d.def_pid_x.Ki);
    pid_x.Kd = parse_edit_or_default(d.edit_kdx, d.def_pid_x.Kd);
    pid_x.N  = parse_edit_or_default(d.edit_nx,  d.def_pid_x.N);

    pid_y = d.def_pid_y;
    pid_y.Kp = parse_edit_or_default(d.edit_kpy, d.def_pid_y.Kp);
    pid_y.Ki = parse_edit_or_default(d.edit_kiy, d.def_pid_y.Ki);
    pid_y.Kd = parse_edit_or_default(d.edit_kdy, d.def_pid_y.Kd);
    pid_y.N  = parse_edit_or_default(d.edit_ny,  d.def_pid_y.N);

    x_ref = val_x_mm / 1000;
    y_ref = val_y_mm / 1000;

    x_ref_vec = zeros(1, d.N);
    y_ref_vec = zeros(1, d.N);
    idx_step = d.t >= d.t_step;
    x_ref_vec(idx_step) = x_ref;
    y_ref_vec(idx_step) = y_ref;

    [x_rk4, y_rk4, vx_rk4, vy_rk4, ux_rk4, uy_rk4] = run_2d_loop( ...
        d.p_x, d.p_y, pid_x, pid_y, d.t, x_ref_vec, y_ref_vec, 'rk4');
    [x_eu, y_eu, ~, ~, ~, ~] = run_2d_loop( ...
        d.p_x, d.p_y, pid_x, pid_y, d.t, x_ref_vec, y_ref_vec, 'euler');

    err_x_mm = (x_ref - x_rk4(end)) * 1000;
    err_y_mm = (y_ref - y_rk4(end)) * 1000;

    fprintf('Target (X, Y)    : (%.2f mm, %.2f mm)\n', val_x_mm, val_y_mm);
    fprintf('Posisi akhir RK4 : (%.2f mm, %.2f mm)\n', x_rk4(end)*1000, y_rk4(end)*1000);
    fprintf('Error akhir RK4  : (%.3f mm, %.3f mm)\n', err_x_mm, err_y_mm);

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

    cla(d.ax3);
    plot(d.ax3, d.t, vx_rk4 * 1000, 'LineWidth', 1.4); hold(d.ax3, 'on');
    plot(d.ax3, d.t, vy_rk4 * 1000, 'LineWidth', 1.4);
    grid(d.ax3, 'on');
    xlabel(d.ax3, 'Waktu (s)');
    ylabel(d.ax3, 'Kecepatan (mm/s)');
    title(d.ax3, 'Kecepatan Linier X & Y (RK4)');
    legend(d.ax3, 'v_x', 'v_y', 'Location', 'best');

    % update teks info
    info_lines = {
        sprintf('t_step  : %.3f s', d.t_step)
        sprintf('Err RK4 : (%.2f, %.2f) mm', err_x_mm, err_y_mm)
        sprintf('Max |u| : (%.1f, %.1f) V', max(abs(ux_rk4)), max(abs(uy_rk4)))
        ''
        sprintf('--- PID X (def: %.3g/%.3g/%.3g) ---', ...
            d.def_pid_x.Kp, d.def_pid_x.Ki, d.def_pid_x.Kd)
        sprintf('Kp=%.3g Ki=%.3g Kd=%.3g N=%.3g', pid_x.Kp, pid_x.Ki, pid_x.Kd, pid_x.N)
        ''
        sprintf('--- PID Y (def: %.3g/%.3g/%.3g) ---', ...
            d.def_pid_y.Kp, d.def_pid_y.Ki, d.def_pid_y.Kd)
        sprintf('Kp=%.3g Ki=%.3g Kd=%.3g N=%.3g', pid_y.Kp, pid_y.Ki, pid_y.Kd, pid_y.N)
        ''
        sprintf('m_x=%.3f m_y=%.3f', d.p_x.m, d.p_y.m)
        sprintf('dt=%.4g t_end=%.2g', d.p_x.dt, d.p_x.t_end)
    };
    d.info_box.String = info_lines;
end

function val = parse_edit_or_default(ctrl, def_val)
    val = str2double(strtrim(ctrl.String));
    if isnan(val) || ~isfinite(val)
        val = def_val;
        ctrl.String = num2str(val);
    end
end

function [x_hist, y_hist, vx_hist, vy_hist, ux_hist, uy_hist] = run_2d_loop( ...
        p_x, p_y, pid_x, pid_y, t, x_ref_vec, y_ref_vec, method)
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
            err_x, p_x.dt, pid_x, pid_state_x, p_x.u_min, p_x.u_max);
        [uy, pid_state_y] = pid_controller( ...
            err_y, p_y.dt, pid_y, pid_state_y, p_y.u_min, p_y.u_max);

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