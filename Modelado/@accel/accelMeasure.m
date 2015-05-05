function accelMmnts = accelMeasure(this,quad)
  %GET ACCELERATION FROM QUAD OBJECT, CHANGE IT TO A BODY REFERENCE FRAME AND ADD NOISE;
  accelMmnts = this.cov * randn(3,1) + quad.curLHb * (-quad.r2pto + [0; 0; 9.81])/9.81;
end
