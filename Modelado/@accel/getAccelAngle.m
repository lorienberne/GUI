function acAng = getAccelAngles(this,quad)
  acMes = this.accelMeasure(quad);
  acAng = [atan2(acMes(2),sqrt(acMes(1)^2 + acMes(3)^2)); atan2(-acMes(1),acMes(3))];
end
