function K_aug = lqr_gain_axis(pa, Q, R)
A = [0,                1,                 0;
    0,               -pa.Beq/pa.Jeq,     pa.Kt/pa.Jeq;
    0,               -pa.Ke/pa.L,       -pa.R/pa.L];
B = [0; 0; 1/pa.L];

A_aug = [A,            zeros(3, 1);
    -pa.rp, 0, 0,  0];
B_aug = [B; 0];

% diskretisasi orde-1
dt = pa.dt;
n  = size(A_aug, 1);
Ad = eye(n) + A_aug * dt;
Bd = B_aug * dt;
Qd = Q * dt;
Rd = R;

% iterasi discrete algebraic riccati equation (DARE)
P = Qd;
max_iter = 5000;
tol = 1e-10;

for iter = 1:max_iter
    BtPB = Bd' * P * Bd + Rd;
    P_next = Ad' * P * Ad - (Ad' * P * Bd) / BtPB * (Bd' * P * Ad) + Qd;
    P_next = 0.5 * (P_next + P_next');

    if norm(P_next - P, 'fro') < tol
        P = P_next;
        break;
    end
    P = P_next;
end

K_aug = (Bd' * P * Bd + Rd) \ (Bd' * P * Ad);
end