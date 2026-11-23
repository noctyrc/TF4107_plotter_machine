clear; clc; close all;

p = params();

p_x = p;
p_x.m   = p.m_x;
p_x.Jeq = p.Jm + p.Jp + (p_x.m * p.rp^2) / p.eta;
p_x.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

p_y = p;
p_y.m   = p.m_y;
p_y.Jeq = p.Jm + p.Jp + (p_y.m * p.rp^2) / p.eta;
p_y.Beq = p.bm + (p.bl * p.rp^2) / p.eta;

Kp_list = [100, 450, 1200, 3500];

run_axis_stability('SUMBU X', p_x, p.pid_pos.x.Ki, p.pid_pos.x.Kd, Kp_list);
run_axis_stability('SUMBU Y', p_y, p.pid_pos.y.Ki, p.pid_pos.y.Kd, Kp_list);

function run_axis_stability(label, pa, Ki, Kd, Kp_list)
A = [0,                1,                 0;
    0,               -pa.Beq/pa.Jeq,     pa.Kt/pa.Jeq;
    0,               -pa.Ke/pa.L,       -pa.R/pa.L];

eig_open = eig(A);
fprintf('\n======================================================\n');
fprintf('ANALISIS KESTABILAN OPEN-LOOP & CLOSED-LOOP - %s\n', label);
fprintf('======================================================\n');
disp('Eigenvalue open-loop:');
disp(eig_open);

fprintf('Uji Eigenvalue Closed-Loop Posisi (Ki = %.1f, Kd = %.1f):\n', Ki, Kd);
for idx = 1:numel(Kp_list)
    Kp = Kp_list(idx);

    A_cl = [0,                  1,                   0,            0;
        0,                 -pa.Beq/pa.Jeq,       pa.Kt/pa.Jeq, 0;
        -Kp*pa.rp/pa.L,    -Kd*pa.rp/pa.L,     -pa.R/pa.L,    Ki/pa.L;
        -pa.rp,              0,                   0,            0];

    ev = eig(A_cl);
    max_real = max(real(ev));
    status = 'STABIL';
    if max_real >= 0
        status = 'TIDAK STABIL / MARGINAL';
    end
    fprintf('Kp = %4.0f | max(Re(lambda)) = %+9.3f | Status: %s\n', Kp, max_real, status);
end
end