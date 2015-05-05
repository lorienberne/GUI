function dir = getGPSDir(this,quad)
    dir = quad.attitSttVect(3) + this.cov(3)*randn(1,1);
end