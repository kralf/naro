function [t, s, v, a, m] = buoyancy_control(n, dt, s_T, v_T)
    model.g = 9.80665;
    model.rho = 1000.0;
    model.A = 0.1;
    model.V = 0.0085;
    model.m_min = 8.4;
    model.m_max = model.m_min+0.3;
    model.dm_max = 0.02;
    model.c = 1.0;
    model.Q = 10e-6;

    controller.K_p = 1e-2;
    controller.K_i = 1e-1;
    controller.K_d = 0.0;
    controller.e_t = 0.0;
    controller.e_T = 0.0;
    
    t = zeros(n, 1);
    s = zeros(n, 1);
    v = zeros(n, 1);
    a = zeros(n, 1);
    m = ones(n, 1)*model.m_min;
    dm = zeros(n, 1);
    
    for i = 2:n
        t(i) = t(1)+i*dt;

        if s(i-1) > s_T+0.25
            v_t = -v_T;
        elseif s(i-1) < s_T-0.25
            v_t = v_T;
        else
            v_t = 0.0;
        end
        
        %dm = buoyancy_pid(controller, dt, s(i-1), s_T);
        dm(i) = buoyancy_pid(controller, dt, v(i-1), v_t);
        if dm(i) < 0.0
            dm(i) = max(dm(i), -model.dm_max*dt);
        else
            dm(i) = min(dm(i), model.dm_max*dt);
        end
        m(i) = min(max(m(i-1)+dm(i), model.m_min), model.m_max);
        [s(i), v(i), a(i)] = buoyancy_state(model, s(i-1), v(i-1), ...
            a(i-1), m(i), dt);
        if s(i) < 0.0
            s(i) = 0.0;
            v(i) = 0.0;
            a(i) = 0.0;
        end
    end
    
    subplot(1, 2, 1);
    plot(t, s, 'color', 'red');
    hold on;
    plot(t, v, 'color', 'green');
    hold on;
    plot(t, a, 'color', 'blue');
    hold on;
    plot(t, m, 'color', 'black');
    xlabel('t');
    legend('s(t)', 'v(t)', 'a(t)', 'm(t)');
    hold off;

    subplot(1, 2, 2);    
    plot(t, dm/dt, 'color', 'red');    
    hold on;
    plot(t, m-model.m_min, 'color', 'green');    
    hold off;
    legend('dm(t)', 'm(t)');
end
