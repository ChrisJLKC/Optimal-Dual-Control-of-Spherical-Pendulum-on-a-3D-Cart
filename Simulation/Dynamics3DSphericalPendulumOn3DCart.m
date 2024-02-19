function [xBarDot, yBarDot] = Dynamics3DSphericalPendulumOn3DCart(m, L, M, g, xBar, yBar, d, ux, uy)
    % X position angle variables.
    Sx = sin(xBar(3));
    Cx = cos(xBar(3));
    % Y position angle variables.
    Sy = sin(yBar(3));
    Cy = cos(yBar(3));
    
    % Denominators of non linear equations for X and Y.
    Dx = m * L^2 * (M + m * (1 * Cx^2));
    Dy = m * L^2 * (M + m * (1 * Cy^2));
    
    %% First Non-linear System.
    xBarDot(1, 1) = xBar(2); % v (velocity) (x)

    % a (acceleration) (x)
    xBarDot(2, 1) = (1/Dx) * (-m^2 * L^2 * g * Cx * Sx + m * L^2 * (m * L * xBar(4)^2 * Sx - d * xBar(2)) + m * L^2 * ux);

    xBarDot(3, 1) = xBar(4);  % v (velocity) (theta)

    % a (acceleration) (Theta)
    xBarDot(4, 1) = (1/Dx) * ((M + m) * m * g * L * Sx - m * L * Cx * (m * L * xBar(4)^2 * Sx - d * xBar(2)) - m * L * Cx * ux);

    
    %% Second Non-linear System.
    yBarDot(1, 1) = yBar(2); % v (velocity) (y)

    % a (acceleration) (y)
    yBarDot(2, 1) = (1/Dy) * (-m^2 * L^2 * g * Cy * Sy + m * L^2 * (m * L * yBar(4)^2 * Sy - d * yBar(2)) + m * L^2 * uy);

    yBarDot(3, 1) = yBar(4);  % v (velocity) (Phi)

    % a (acceleration) (Phi)
    yBarDot(4, 1) = (1/Dy) * ((M + m) * m * g * L * Sy - m * L * Cy * (m * L * yBar(4)^2 * Sy - d * yBar(2)) - m * L * Cy * uy);
end