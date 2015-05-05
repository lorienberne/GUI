classdef accel
  properties
    cov = 0; %ACCELEROMETER NOISE COVARIANCE
  end

  methods
    function this = accel(cov)
      this.cov = cov;
    end

    accelMmnts = accelMeasure(this,quad);
  end

end
