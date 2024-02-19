%% Testing Kinematics.
function SphericalPendulumKinematics()
% Stating variables.
theta = pi;
phi = pi;
x = 75.5;
y = 100;
z = 0;

% Length of pendulum rod.
L = 0.4;

% Kinematics.
xm = x + L * sin(theta);
zm = -L * cos(theta);
ym = y + L * sin(phi);

xm
ym
zm

% Displaying figure.
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