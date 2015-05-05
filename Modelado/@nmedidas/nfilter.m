function this = nfilter(this, new, derNew)
  for i = this.numMed:-1:2
    this.medidas(i,:)    = this.medidas(i-1,:);
    this.derMedidas(i,:) = this.derMedidas(i-1,:);
  end
  this.medidas(1,:)    = new;
  this.derMedidas(1,:) = derNew; 
end
