function dm = buoyancy_pid(controller, dt, s_t, s_T)
    e_t = s_T-s_t;
    controller.e_T = controller.e_T+e_t*dt;
    de_t = (e_t-controller.e_t)/dt;
    controller.e_t = e_t;
    
    dm = (controller.K_p*e_t+controller.K_i*controller.e_T+ ...
        controller.K_d*de_t);
end
