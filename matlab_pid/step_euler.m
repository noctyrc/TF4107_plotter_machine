function x_next = step_euler(ode_fun, t, x, u, dt)
dx = ode_fun(t, x, u);
x_next = x + dt * dx;
end