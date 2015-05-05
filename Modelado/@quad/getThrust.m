function thrust = getThrust(quad)
    thrust =  [quad.kProp(1) * quad.rotorOmega(1)^2;
               quad.kProp(1) * quad.rotorOmega(2)^2;
               quad.kProp(1) * quad.rotorOmega(3)^2;
               quad.kProp(1) * quad.rotorOmega(4)^2];
end
