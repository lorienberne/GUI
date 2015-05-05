function mgMmnt = magMeasure(this,quad)
  mgMmnt = this.cov * randn(1,1) + quad.attitSttVect(3);
end
