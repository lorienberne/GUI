classdef nmedidas
  properties
    numMed;
    medidas;
    derMedidas;
    dt;
    weights;
  end
  methods
    function this = nmedidas(numMed,dim,dt)
      this.numMed     = numMed;
      this.medidas    = zeros(numMed,dim);
      this.derMedidas = zeros(numMed,dim);
      this.dt         = dt;
      this.weights    = (0:(numMed-1))';
    end
    this = nfilter(this, new, derNew);
    med  = getNMes(this);
  end
end
