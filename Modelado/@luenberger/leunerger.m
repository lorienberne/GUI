classdef luenberger
  properties
    K;
  end

  methods
    function this = luenberger(K)
    end

    estimate = updateLuen();
  end

end
