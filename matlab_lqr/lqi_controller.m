function [u, q_out] = lqi_controller(pos_ref, vel_ref, state, rp, K_aug, q_in, dt, u_min, u_max)
pos_curr = rp * state(1);
vel_curr = rp * state(2);
i_curr   = state(3);

err_pos = pos_ref - pos_curr;
err_vel = vel_ref - vel_curr;

q_cand = q_in + err_pos * dt;

k_pos = K_aug(1) / rp;
k_vel = K_aug(2) / rp;
k_cur = K_aug(3);
k_int = -K_aug(4);

u_raw = k_pos * err_pos + k_vel * err_vel - k_cur * i_curr + k_int * q_cand;
u = max(u_min, min(u_max, u_raw));

if (u == u_raw) || ...
        (u == u_max && err_pos < 0) || ...
        (u == u_min && err_pos > 0)
    q_out = q_cand;
else
    q_out = q_in;
end
end