clc
close all

a=1;
b=1;
c=-8;
hold on
grid on
plot(out.x2,out.y2,'LineWidth',2);
hold on
plot(10,-2,'o','LineWidth',2)
hold on
plot(4,4,'o','LineWidth',2)
title('La trayectoria del robot')
legend('Trayectoria CLR')
xlabel('X')
ylabel('Y')
