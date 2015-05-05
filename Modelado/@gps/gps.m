classdef gps
  properties
    cov;
  end

  methods
    function this = gps(cov)
      this.cov = cov;
    end
    gpsMes = gpsMeasure(this,q);
  end

end
