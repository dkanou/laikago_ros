t = -2:1e-3:2;
w = 10;
y0 = tanh(w*t);
y1 = atan(w*t);

figure(1)
plot(t, [y0; y1])


t2 = 0:1e-3:1;
w2 = 1.3;
y2 = sin(2*pi*w2*t2);
delta = 0.1;
delta2 = 0.2;
y3 = (1/atan(1/delta))*atan(sin(2*pi*t2*w2)/delta);
y4 = (1/tanh(1/delta2))*tanh(sin(2*pi*t2*w2)/delta2);

figure(2)
plot(t2, y2)
hold on
plot(t2, sign(y2))
plot(t2, [y3; y4])
hold off
grid on


%%
t=linspace(0,2*pi,500);
delta = 0.1;
A = 1;
f = 1/(2*pi);
y1 = A*sin(2*pi*t*f);
y2 = (2*A/pi)*atan(sin(2*pi*t*f)/delta);
y3 = (A/atan(1/delta))*atan(sin(2*pi*t*f)/delta);

figure; hold on;
h1=plot(t,y1,'r-','LineWidth',5);
h2=plot(t,y2,'g-','LineWidth',4);
h3=plot(t,y3,'b-','LineWidth',2);
legend([h1 h2 h3],{'Sine','Smooth square wave','Corrected smooth square wave'},'Location','SouthOutSide');
axis tight; axis square;
set(gca,'FontSize',15); grid on; box on;
drawnow;