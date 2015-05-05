function k = kCalc(this)
    k = this.pPriori * (this.C)' * pinv(this.C * this.pPriori * (this.C)' + this.R);
end
