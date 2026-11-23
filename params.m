function p = params()
% Parameter mekanik
p.m_x     = 0.758;      % kg (massa bergerak sumbu X)
p.m_y     = 0.328;      % kg (massa bergerak sumbu Y)
p.m       = p.m_x;      % kg (massa aktif yang disimulasikan)
p.rp      = 0.0064;     % m (jari-jari pitch pulley GT2 20T)
p.eta     = 0.88;       % efisiensi mekanis transmisi belt GT2
p.Jm      = 5.4e-6;     % kg*m^2 (momen inersia rotor NEMA 17)
p.Jp      = 1.5e-7;     % kg*m^2 (momen inersia pulley)
p.bm      = 1.0e-4;     % N*m*s/rad (redaman viskos rotor)
p.bl      = 2.0;        % N*s/m (gesek viskos pemandu linier)
p.mu_k    = 0.03;       % koefisien gesek Coulomb
p.k_cog   = 1.05;       % faktor koreksi efek offset CoG
p.g       = 9.81;       % m/s^2
p.F_pen   = 1.50;       % N (gaya seret ujung pena saat turun)
p.pen_down = true;      % true = pena menyentuh media

% Parameter elektrik motor stepper 
p.R       = 1.5;        % Ohm
p.L       = 2.8e-3;     % H
p.Kt      = 0.235;      % N*m/A
p.Ke      = 0.235;      % V*s/rad

% Parameter beban terrefleksi ke poros motor
p.Jeq     = p.Jm + p.Jp + (p.m * p.rp^2) / p.eta;
p.Beq     = p.bm + (p.bl * p.rp^2) / p.eta;

% Batasan aktuator & parameter waktu simulasi
p.u_min   = -24.0;      % V (batas bawah tegangan/sinyal kontrol)
p.u_max   = 24.0;       % V (batas atas tegangan/sinyal kontrol)
p.dt      = 0.001;      % s (1 ms)
p.t_end   = 2.0;        % s

% Parameter PID kecepatan (speed control)
p.pid_speed.Kp = 15.0;
p.pid_speed.Ki = 120.0;
p.pid_speed.Kd = 0.05;
p.pid_speed.N  = 100.0;     % koefisien filter derivatif (rad/s)

% Parameter PID posisi (position control)
p.pid_pos.Kp   = 450.0;
p.pid_pos.Ki   = 35.0;
p.pid_pos.Kd   = 28.0;
p.pid_pos.N    = 150.0;     % koefisien filter derivatif (rad/s)
end