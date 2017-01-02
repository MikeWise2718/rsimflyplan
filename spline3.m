clf;
n = 5;
x = 0:1:n;
y = [cumsum(rand(1,n+1)); cumsum(rand(1,n+1));  cumsum(rand(1,n+1))];
% y = cumsum(rand(1,n+1));
o3 = [0;0;0];
pp = spline(x,[o3,y,o3] );
xx = 0:.1:n;
yy = ppval(pp,xx);

ppd = pp;
M = [0 3 0 0;0 0 2 0;0 0 0 1;0 0 0 0];
ppd.coefs = ppd.coefs*M;
yyd = ppval(ppd,xx);

ppdd = ppd;
ppdd.coefs = ppdd.coefs*M;
yydd = ppval(ppdd,xx);

ppddd = ppdd;
ppddd.coefs = ppddd.coefs*M;
yyddd = ppval(ppddd,xx);

ppdddd = ppddd;
ppdddd.coefs = ppdddd.coefs*M;
yydddd = ppval(ppdddd,xx);

%% Now plot them
figure
subplot(5,1,1)
plot(x,y,'o',xx,yy);
title('Splined values - Position')

subplot(5,1,2)
plot(xx,yyd);
title('1st Derivative - Velocity')

subplot(5,1,3)
plot(xx,yydd);
title('2nd Derivative - Acceleration')

subplot(5,1,4)
plot(xx,yyddd);
title('3rd Derivative - Jerk')


subplot(5,1,5)
plot(xx,yydddd);
title('4th Derivative - Snap or Jounce')