clear all;
t = pathGen();
t = t.generatePath([5 -5 0; 5 0 2; 5 5 -1; 0 5 0]);
l = 0:0.01:1;
x1 = t.coefs(1,1)*l.^3 + t.coefs(1,2)*l.^2 + t.coefs(1,3)*l + t.coefs(1,4);
x2 = t.coefs(1,5)*l.^3 + t.coefs(1,6)*l.^2 + t.coefs(1,7)*l + t.coefs(1,8);
x3 = t.coefs(1,9)*l.^3 + t.coefs(1,10)*l.^2 + t.coefs(1,11)*l + t.coefs(1,12);
x4 = t.coefs(1,13)*l.^3 + t.coefs(1,14)*l.^2 + t.coefs(1,15)*l + t.coefs(1,16);


y1 = t.coefs(2,1)*l.^3 + t.coefs(2,2)*l.^2 + t.coefs(2,3)*l + t.coefs(2,4);
y2 = t.coefs(2,5)*l.^3 + t.coefs(2,6)*l.^2 + t.coefs(2,7)*l + t.coefs(2,8);
y3 = t.coefs(2,9)*l.^3 + t.coefs(2,10)*l.^2 + t.coefs(2,11)*l + t.coefs(2,12);
y4 = t.coefs(2,13)*l.^3 + t.coefs(2,14)*l.^2 + t.coefs(2,15)*l + t.coefs(2,16);


z1 = t.coefs(3,1)*l.^3 + t.coefs(3,2)*l.^2 + t.coefs(3,3)*l + t.coefs(3,4);
z2 = t.coefs(3,5)*l.^3 + t.coefs(3,6)*l.^2 + t.coefs(3,7)*l + t.coefs(3,8);
z3 = t.coefs(3,9)*l.^3 + t.coefs(3,10)*l.^2 + t.coefs(3,11)*l + t.coefs(3,12);
z4 = t.coefs(3,13)*l.^3 + t.coefs(3,14)*l.^2 + t.coefs(3,15)*l + t.coefs(3,16);


plot3(x1,y1,z1);
hold on;
plot3(x2,y2,z2);
plot3(x3,y3,z3);
plot3(x4,y4,z4);
legend('1','2','3','4');
hold off;
