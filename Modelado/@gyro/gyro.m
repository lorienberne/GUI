classdef gyro
  properties
    cov = 0;
  end

  methods
    function this = gyro(cov)
      this.cov = cov;
    end
  end

end
