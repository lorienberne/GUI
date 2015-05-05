%% CLASS THAT MODELS THE MAGNETOMETER ONBOARD OF THE QUADCOPTER
classdef magnet
  properties
    cov = 0;
  end
  methods
    function this = magnet(cov)
      this.cov = cov;
    end
    mgMmnt = magMeasure(this,quad);
  end
end
