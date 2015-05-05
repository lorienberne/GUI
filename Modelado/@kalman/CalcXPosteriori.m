function xPost = CalcXPosteriori(this, y)

    xPost = this.xPriori + this.K * (y - this.C * this.xPriori);

end
