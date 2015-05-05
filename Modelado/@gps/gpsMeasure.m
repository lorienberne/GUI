function gpsMes = gpsMeasure(this,q)
  gpsMes =[this.cov(1) * randn(3,1); this.cov(2) * randn(3,1)]  + q.posSttVect;
end
