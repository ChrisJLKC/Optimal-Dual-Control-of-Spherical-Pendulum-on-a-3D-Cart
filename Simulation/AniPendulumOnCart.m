function AniPendulumOnCart(Mx, My, xm, ym, zm, t)
    % Size of Cart and wheels.
    SC = 0.1;
    wr = 0.05;
    
    % wheel spheres.
    [X, Y, Z] = sphere;

    wrX = X * wr;
    wrY = Y * wr;
    wrZ = Z * wr;

    % Setting time step wait.
    timeStep = t(length(t)) / length(t);

    %% Animation.
    figure;
    for nidx = 1:(length(t) - 1)
        % Clear figure.
        clf;
        % Set position of cube.
        Cube_Pos = [(Mx(nidx)-SC) (My(nidx)-SC) 0;
                    (Mx(nidx)+SC) (My(nidx)-SC) 0;
                    (Mx(nidx)+SC) (My(nidx)+SC) 0;
                    (Mx(nidx)-SC) (My(nidx)+SC) 0;
                    (Mx(nidx)-SC) (My(nidx)-SC) -SC*2;
                    (Mx(nidx)+SC) (My(nidx)-SC) -SC*2;
                    (Mx(nidx)+SC) (My(nidx)+SC) -SC*2;
                    (Mx(nidx)-SC) (My(nidx)+SC) -SC*2];
        cube_x = Cube_Pos(:, 1);
        cube_y = Cube_Pos(:, 2);
        cube_z = Cube_Pos(:, 3);

        idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';
        hold on;
        % Plot cube.
        patch(cube_x(idx), cube_y(idx), cube_z(idx), 'r');
        % Plot wheels.
        surf(wrX+Cube_Pos(5,1), wrY+Cube_Pos(5,2), wrZ-SC*2);
        surf(wrX+Cube_Pos(6,1), wrY+Cube_Pos(6,2), wrZ-SC*2);
        surf(wrX+Cube_Pos(7,1), wrY+Cube_Pos(7,2), wrZ-SC*2);
        surf(wrX+Cube_Pos(8,1), wrY+Cube_Pos(8,2), wrZ-SC*2);
        % Plot floor lines.
        plot3([-2 2], [0 0], [-0.25 -0.25], 'b-');
        plot3([0 0], [-2 2], [-0.25 -0.25], 'b-');
        % Plot pendulum line.
        plot3([Mx(nidx) xm(nidx)], [My(nidx) zm(nidx)], [0 ym(nidx)], 'bO-', 'LineWidth', 2, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');

        title("Spherical Pendulum on a Cart");
        xlabel("x[m]");
        ylabel("y[m]");
        zlabel("z[m]");
        xlim([-1 1]);
        ylim([-1 1]);
        zlim([-1 1]);
        axis square;
        view(40, 40);

        % Wait time step.
        pause(timeStep);
    end
    hold off;
end