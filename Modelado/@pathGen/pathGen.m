classdef pathGen
  properties
    coefs;
  end
  methods
    function this = pathGen()
    end
    this = generatePath(this,points);
    error = getError(position);
  end
end
