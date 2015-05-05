function this = updateKalman(this, u, mState)
  this.xPriori     = this.CalcXPriori(u);
  this.pPriori     = this.CalcPPriori();
  this.K           = this.CalcK();
  this.xPosteriori = this.CalcXPosteriori(mState);
  this.pPosteriori = this.CalcPPosteriori();
end
