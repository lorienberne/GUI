function med = getNMes(this)
    med = [(sum(this.medidas + diag(this.weights)*this.derMedidas.*this.dt,1)/this.numMed)';this.derMedidas(1,:)'];
end