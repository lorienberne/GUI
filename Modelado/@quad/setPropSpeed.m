function this = setPropSpeed(this,attSignal, posSignal)
%   this.rotorOmega = [509.35 + sign(posSignal)*sqrt(abs(posSignal)/4) + sign(attSignal(3))*sqrt(abs(attSignal(3))/2) + sign(attSignal(1))*sqrt(abs(attSignal(1))/4);
%                      509.35 + sign(posSignal)*sqrt(abs(posSignal)/4) - sign(attSignal(2))*sqrt(abs(attSignal(2))/2) - sign(attSignal(1))*sqrt(abs(attSignal(1))/4);
%                      509.35 + sign(posSignal)*sqrt(abs(posSignal)/4) - sign(attSignal(3))*sqrt(abs(attSignal(3))/2) + sign(attSignal(1))*sqrt(abs(attSignal(1))/4);
%                      509.35 + sign(posSignal)*sqrt(abs(posSignal)/4) + sign(attSignal(2))*sqrt(abs(attSignal(2))/2) - sign(attSignal(1))*sqrt(abs(attSignal(1))/4)];

omega1 = 509.35^2 + posSignal/4 + attSignal(1)/2 + attSignal(3)/4;
omega3 = 509.35^2 + posSignal/4 - attSignal(1)/2 + attSignal(3)/4;

omega4 = 509.35^2 + posSignal/4 + attSignal(2)/2 - attSignal(3)/4;
omega2 = 509.35^2 + posSignal/4 - attSignal(2)/2 - attSignal(3)/4;

if omega1 < 0
    omega1 = 0;
end

if omega2 < 0
    omega2 = 0;
end

if omega3 < 0
    omega3 = 0;
end

if omega4 < 0
    omega4 = 0;
end

if omega1 > 837^2
    omega1 = 837^2;
end

if omega2 > 837^2
    omega2 = 837^2;
end

if omega3 > 837^2
    omega3 = 837^2;
end

if omega4 > 837^2
    omega4 = 837^2;
end
this.rotorOmega = [sqrt(omega1);
                   sqrt(omega2);
                   sqrt(omega3);
                   sqrt(omega4)];

end
