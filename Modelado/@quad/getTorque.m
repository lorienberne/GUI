%% tor = getTorque(quad)
% GETS A QUAD OBJECT AND RETURNS THE TORQUE VECTOR, RESULT OF THE ENGINES THRUST

function tor = getTorque(quad)
% GET THE CURRENT THRUST OF EACH MOTOR
  T = quad.getThrust();

% CALCULATE THE TORQUE
  tor = [                             (T(1) - T(3)) * quad.l;
                                      (T(4) - T(2)) * quad.l;
       quad.kProp(2) * (quad.rotorOmega(1)^2 - quad.rotorOmega(2)^2 + quad.rotorOmega(3)^2 - quad.rotorOmega(4)^2)];

end
