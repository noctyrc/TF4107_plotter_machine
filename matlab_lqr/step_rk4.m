function x_next = step_rk4(ode_fun, t, x, u, dt)
k1 = ode_fun(t,              x,                  u);
k2 = ode_fun(t + 0.5 * dt,   x + 0.5 * dt * k1,  u);
k3 = ode_fun(t + 0.5 * dt,   x + 0.5 * dt * k2,  u);
k4 = ode_fun(t + dt,         x + dt * k3,        u);

x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
end