function [obj,this] = getObj(this,currPos,speed)
    if(this.flag == 1)
        if(norm(this.points(this.currSpline + 1,:)' - currPos) < 1)
            this.currSpline = this.currSpline + 1;
            this.flag = 0;
        end
    end
    
    if(norm(this.points(this.currSpline,:)' - currPos) > 2)
        this.flag = 1;
    end
    syms t;
    
	x = 3*this.coefs(1,(this.currSpline - 1)*4 + 1)*t^2 + 2*this.coefs(1,(this.currSpline - 1)*4 + 2)*t + this.coefs(1,(this.currSpline - 1)*4 + 3);
	y = 3*this.coefs(2,(this.currSpline - 1)*4 + 1)*t^2 + 2*this.coefs(2,(this.currSpline - 1)*4 + 2)*t + this.coefs(2,(this.currSpline - 1)*4 + 3);
	z = 3*this.coefs(3,(this.currSpline - 1)*4 + 1)*t^2 + 2*this.coefs(3,(this.currSpline - 1)*4 + 2)*t + this.coefs(3,(this.currSpline - 1)*4 + 3);
    tg = [x;y;z]/norm([x;y;z]);
    
    xc = this.coefs(1,(this.currSpline - 1)*4 + 1)*t^3 + this.coefs(1,(this.currSpline - 1)*4 + 2)*t^2 + this.coefs(1,(this.currSpline - 1)*4 + 3)*t + this.coefs(1,(this.currSpline - 1)*4 + 4);
    yc = this.coefs(2,(this.currSpline - 1)*4 + 1)*t^3 + this.coefs(2,(this.currSpline - 1)*4 + 2)*t^2 + this.coefs(2,(this.currSpline - 1)*4 + 3)*t + this.coefs(2,(this.currSpline - 1)*4 + 4);
    zc = this.coefs(3,(this.currSpline - 1)*4 + 1)*t^3 + this.coefs(3,(this.currSpline - 1)*4 + 2)*t^2 + this.coefs(3,(this.currSpline - 1)*4 + 3)*t + this.coefs(3,(this.currSpline - 1)*4 + 4);

    sol = vpasolve([currPos(1)-xc,currPos(2)-yc,currPos(3)-zc]*tg == 0);
    
    speedVect = speed*subs(tg,sol);
    obj = double([subs(xc,sol);subs(yc,sol);subs(zc,sol);speedVect]);
end
