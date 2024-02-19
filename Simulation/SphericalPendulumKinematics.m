function SphericalPendulumKinematics()

theta = pi;
phi = pi;
x = 75.5;
y = 100;
z = 0;

L = 0.4;

xm = x + L * sin(theta);
zm = -L * cos(theta);
ym = y + L * sin(phi);

xm
ym
zm

figure
hold on
view(40, 40);
axis equal
axis square
plot3([x xm], [y ym], [z zm], 'r-o');
zlabel("z");
ylabel("y");
xlabel("x");
hold off;

end