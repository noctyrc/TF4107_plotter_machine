function dx = dynamics_model(~, x, u, p)
% State: x = [theta_m; omega_m; i_a]
%   theta_m : posisi sudut rotor motor [rad]
%   omega_m : kecepatan sudut rotor motor [rad/s]
%   i_a     : arus kumparan efektif [A]
theta_m = x(1);
omega_m = x(2);
i_a     = x(3);

% Saturasi tegangan input aktuator
V = max(p.u_min, min(p.u_max, u));

% Kecepatan linier & gesekan nonlinier (Coulomb + beban pena)
v_lin   = p.rp * omega_m;
sgn_v   = tanh(200.0 * v_lin);
F_coul  = p.k_cog * p.mu_k * p.m * p.g * sgn_v;
F_ext   = 0.0;
if p.pen_down
    F_ext = p.F_pen * sgn_v;
end
tau_dist = (p.rp / p.eta) * (F_coul + F_ext);

% Persamaan diferensial terintegrasi
dtheta_m = omega_m;
domega_m = (p.Kt * i_a - p.Beq * omega_m - tau_dist) / p.Jeq;
di_a     = (V - p.R * i_a - p.Ke * omega_m) / p.L;

dx = [dtheta_m; domega_m; di_a];
end