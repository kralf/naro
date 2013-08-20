function [s, v, a] = buoyancy_state(model, s, v, a, m, dt)
  a = velocity_to_acceleration(model, v, m);
  v = v+a*dt;
  s = s+v*dt;
end

function F = velocity_to_force(model, v, m)
  F_g = m*model.g;
  F_b = model.V*model.rho*model.g;
  F_c = 0.5*v*v*model.rho*model.A*model.c;

  if v > 0.0
    F = F_g-F_b-F_c;
  else
    F = F_g-F_b+F_c;
  end
end

function a = velocity_to_acceleration(model, v, m)
    a = velocity_to_force(model, v, m)/m;
end
