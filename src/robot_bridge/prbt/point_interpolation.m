p1 = [0.01, 0.0001];
p2 = [0.05, 0.005];
p3 = [0.1, 0.02];
p4 = [0.5, 0.1];

x = [p1(1) p2(1) p3(1) p4(1)];
y = [p1(2) p2(2) p3(2) p4(2)];

p1 = polyfit(x,y,1);
p2 = polyfit(x,y,2);
x1 = linspace(-5,5);
f1 = polyval(p1,x1);
f2 = polyval(p2,x1);

figure;
plot(x,y,'o');
hold on;
plot (x1,f1);
plot (x1,f2);
legend('y','f1','f2');

