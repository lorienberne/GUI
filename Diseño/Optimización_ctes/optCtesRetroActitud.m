syms xd;
syms t s;
syms k1 k2;

Xd = 1;
ke = 10000;
kv = 10;
ku = 100;


x = subs(ilaplace(((xd*k1)/s)/(s^2+s*k2+k1)),xd,Xd);
e = x-Xd;
u = k2*diff(x,t) + k1*e;

fc = ke*int(e^2,t,0,10) + kv*int(diff(x,t)^2,t,0,10) + ku*int(u^2,t,0,10);


K1 = -0.01;
K2 = -0.01;
for i=0:3000
    dk1 = double(subs(subs(diff(fc,k1),k1,K1),k2,K2));
    dk2 = double(subs(subs(diff(fc,k2),k1,K1),k2,K2));

    K1 = K1 - 0.00001*dk1;
    K2 = K2 - 0.00001*dk2;

end


f = subs(subs(x,k1,K1),k2,K2);
ezplot(f,[0 10]);
axis([-5 2 0 5]);

K1
K2

K = [0 0 K1/0.000212 0 0 K2/0.000212;
     0 K1/0.000212 0 0 K2/0.000212 0;
     K1/0.0000638 0 0 K2/0.0000638 0 0];