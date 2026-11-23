function [u_sat, state_out] = pid_controller(err, dt, cfg, state_in, u_min, u_max)
if k_is_empty(state_in)
    state_in = struct('integral', 0.0, 'd_filt', 0.0, 'prev_err', err);
end

% Proporsional
P = cfg.Kp * err;

% Derivatif
raw_d = (err - state_in.prev_err) / dt;
alpha = (cfg.N * dt) / (1.0 + cfg.N * dt);
d_filt = state_in.d_filt + alpha * (raw_d - state_in.d_filt);
D = cfg.Kd * d_filt;

% Kandidat integral & output sebelum saturasi
I_cand = state_in.integral + cfg.Ki * err * dt;
u_unsat = P + I_cand + D;

% Saturasi output aktuator
u_sat = max(u_min, min(u_max, u_unsat));

% Conditional integration
if (u_unsat == u_sat) || ...
        (u_unsat > u_max && err < 0) || ...
        (u_unsat < u_min && err > 0)
    I_new = I_cand;
else
    I_new = state_in.integral;
end

state_out = struct( ...
    'integral', I_new, ...
    'd_filt', d_filt, ...
    'prev_err', err ...
    );
end

function tf_val = k_is_empty(s)
tf_val = isempty(s) || ~isstruct(s) || ~isfield(s, 'integral');
end