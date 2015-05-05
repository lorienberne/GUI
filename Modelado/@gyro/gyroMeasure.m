function measurement = gyroMeasure(this,quad)
    measurement = this.cov * randn(3,1) + quad.attitSttVect(4:6,1);
end
