function pPri = CalcPPriori(this)
  pPri = this.A * this.pPosteriori * (this.A)' + this.Q;
end
