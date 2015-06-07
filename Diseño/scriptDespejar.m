syms xp yp zp
syms ax bx cx dx
syms ay by cy dy
syms az bz cz dz
syms t

xc = ax*t^3 + bx*t^2 + cx*t + dx;
yc = ay*t^3 + by*t^2 + cy*t + dy;
zc = az*t^3 + bz*t^2 + cz*t + dz;

xt = 3*ax*t^2 + 2*bx*t^2 + cx;
yt = 3*ay*t^2 + 2*by*t^2 + cy;
zt = 3*az*t^2 + 2*bz*t^2 + cz;

sol = solve([xp-xc,yp-yc,zp-zc]*[xt;yt;zt], 't');