clf;
x = 0:1:10;
y = cumsum(rand(1,11));
pp = spline(x,y);
xx = 0:.1:10;
yy = ppval(pp,xx);

ppd = pp;
M = [0 3 0 0;0 0 2 0;0 0 0 1;0 0 0 0];
ppd.coefs = ppd.coefs*M;
yyd = ppval(ppd,xx);

figure
subplot(2,1,1)
plot(x,y,'o',xx,yy);
title('Splined values')

subplot(2,1,2)
plot(xx,yyd);
title('Derivitives')
