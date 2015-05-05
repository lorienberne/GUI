function pPost = CalPPosteriori(this)
  pPost = (eye(length(this.C)) - this.K * this.C) * this.pPriori;
end
