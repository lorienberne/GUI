function plotPath(this, fig, axes)
    set(fig,'CurrentAxes',axes);
    numPoints = size(this.coefs);
    t = 0:0.01:1;
    for i = 1:numPoints(1,2)/4
        x = this.coefs(1,(i-1)*4 + 1)*t.^3 + this.coefs(1,(i-1)*4 + 2)*t.^2 + this.coefs(1,(i-1)*4 + 3)*t + this.coefs(1,(i-1)*4 + 4);
        y = this.coefs(2,(i-1)*4 + 1)*t.^3 + this.coefs(2,(i-1)*4 + 2)*t.^2 + this.coefs(2,(i-1)*4 + 3)*t + this.coefs(2,(i-1)*4 + 4);
        z = this.coefs(3,(i-1)*4 + 1)*t.^3 + this.coefs(3,(i-1)*4 + 2)*t.^2 + this.coefs(3,(i-1)*4 + 3)*t + this.coefs(3,(i-1)*4 + 4);
        hold on;
        plot3(x,y,z);
    end
    
end 