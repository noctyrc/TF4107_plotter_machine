clear; clc; close all;

p = params();
t = 0:p.dt:p.t_end;
N = numel(t);

% target awal kecepatan [mm/s]
init_vx_mm_s = 120.0; % mm/s (0.120 m/s)
init_vy_mm_s = 80.0;  % mm/s (0.080 m/s)

% nilai awal PID speed diambil dari params()
def_pid_x = p.pid_speed.x;
def_pid_y = p.pid_speed.y;

% parameter sumbu X & sumbu Y
p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

fig = figure('Name', '2-Axis Speed Control Simulation', 'Position', [90, 80, 1200, 720]);

% grafik kecepatan linier X Y
ax1 = subplot(2, 1, 1);
ax1.Position = [0.07, 0.56, 0.58, 0.35];
grid(ax1, 'on');
ylabel(ax1, 'Kecepatan (mm/s)');
title(ax1, 'Kontrol Kecepatan Sumbu X & Y Plotter (RK4)');

% grafik tegangan input u_x u_y
ax2 = subplot(2, 1, 2);
ax2.Position = [0.07, 0.11, 0.58, 0.35];
grid(ax2, 'on');
xlabel(ax2, 'Waktu (s)');
ylabel(ax2, 'Tegangan Input u (V)');

% panel input dan informasi samping
panel = uipanel(fig, ...
    'Title', 'Target & Parameter PID Speed', ...
    'FontWeight', 'bold', ...
    'Position', [0.68, 0.08, 0.29, 0.84]);

% input vx_ref [mm/s]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.90, 0.44, 0.045], ...
    'String', 'vx_ref (mm/s):', 'HorizontalAlignment', 'left');
edit_vx = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.51, 0.90, 0.44, 0.05], ...
    'String', num2str(init_vx_mm_s));

% input vy_ref [mm/s]
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.84, 0.44, 0.045], ...
    'String', 'vy_ref (mm/s):', 'HorizontalAlignment', 'left');
edit_vy = uicontrol(panel, 'Style', 'edit', ...
    'Units', 'normalized', 'Position', [0.51, 0.84, 0.44, 0.05], ...
    'String', num2str(init_vy_mm_s));

% header kolom PID speed X & Y
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.78, 0.36, 0.04], ...
    'String', 'Gain PID', 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.44, 0.78, 0.24, 0.04], ...
    'String', 'Sumbu X', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.71, 0.78, 0.24, 0.04], ...
    'String', 'Sumbu Y', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% helper membuat baris input PID X dan Y
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

% kotak teks ringkasan hasil & gain PID di bawah tombol
info_box = uicontrol(panel, 'Style', 'text', ...
    'Units', 'normalized', 'Position', [0.05, 0.02, 0.90, 0.44], ...
    'HorizontalAlignment', 'left', ...
    'FontName', 'FixedWidth', ...
    'FontSize', 8.5, ...
    'String', '');

sim_data = struct('p', p, 'p_x', p_x, 'p_y', p_y, 't', t, 'N', N, ...
    'ax1', ax1, 'ax2', ax2, ...
    'edit_vx', edit_vx, 'edit_vy', edit_vy, ...
    'edit_kpx', edit_kpx, 'edit_kix', edit_kix, 'edit_kdx', edit_kdx, 'edit_nx', edit_nx, ...
    'edit_kpy', edit_kpy, 'edit_kiy', edit_kiy, 'edit_kdy', edit_kdy, 'edit_ny', edit_ny, ...
    'def_pid_x', def_pid_x, 'def_pid_y', def_pid_y, ...
    'info_box', info_box, 'def_vx_mm_s', init_vx_mm_s, 'def_vy_mm_s', init_vy_mm_s);

all_edits = [edit_vx, edit_vy, edit_kpx, edit_kix, edit_kdx, edit_nx, ...
             edit_kpy, edit_kiy, edit_kdy, edit_ny];
btn_run.Callback = @(~, ~) update_speed_simulation(sim_data);
for h = all_edits
    h.Callback = @(~, ~) update_speed_simulation(sim_data);
end

update_speed_simulation(sim_data);

function update_speed_simulation(d)
    val_vx_mm_s = parse_edit_or_default(d.edit_vx, d.def_vx_mm_s);
    val_vy_mm_s = parse_edit_or_default(d.edit_vy, d.def_vy_mm_s);

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
            err_x, d.p_x.dt, pid_x, pid_state_x, d.p_x.u_min, d.p_x.u_max);
        [uy, pid_state_y] = pid_controller( ...
            err_y, d.p_y.dt, pid_y, pid_state_y, d.p_y.u_min, d.p_y.u_max);

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
        sprintf('Max|u| : (%.1f, %.1f) V', max(abs(ux_hist)), max(abs(uy_hist)))
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