classdef pathGen
  properties
    coefs;
	points;
    currSpline;
    flag = 0;
  end
  methods
    function this = pathGen()
        this.currSpline = 1;
    end
    this = generatePath(this, points, type);
    [obj,this] = getObj(coefs,currPos,speed);
    plotPath(this, fig, axes);
  end
end
