function [t, s, v, a, m] = buoyancy_simulation(n, dt)
    model.g = 9.80665;
    model.rho = 1000.0;
    model.A = 0.1;
    model.V = 0.0085;
    model.m_min = 8.6;
    model.c = 1.0;
    model.Q = 10e-6;

    t = zeros(n, 1);
    s = zeros(n, 1);
    v = zeros(n, 1);
    a = zeros(n, 1);
    m = ones(n, 1)*model.m_min;
    
    for i = 2:n
        t(i) = t(1)+i*dt;
        [s(i), v(i), a(i)] = buoyancy_state(model, s(i-1), v(i-1), ...
            a(i-1), m(i-1), dt);
    end
    
    plot(t, s, 'color', 'red');
    hold on;
    plot(t, v, 'color', 'green');
    hold on;
    plot(t, a, 'color', 'blue');
    xlabel('t');
    legend('s(t)', 'v(t)', 'a(t)');
    hold off;
end
