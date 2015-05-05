classdef fback
  properties
    K = 0;
  end
  methods
    function this = fback(K)
      this.K = K;
    end
    signal = getControlSignal(this, kalman,state, desiredState);
  end
end
