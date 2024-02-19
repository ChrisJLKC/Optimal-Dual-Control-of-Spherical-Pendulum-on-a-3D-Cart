function [Mx, My, xm, ym, zm, t] = CalcPendUncontrolled()
    % Display Figures.
    ShowFig = 0;

    % Setting Parameters.
    m = 1;
    M = 2;
    g = -9.81;
    L = 0.4;
    d = 10;
    
    % Set time step for amount of calculation time.
    h = 0.025;
    t = 0:h:20;
    
    % Starting position of pendulum in relation to X and Y.
    xBar0 = [0; 0; (-pi - 0.6); 0];
    yBar0 = [0; 0; (-pi + 0.4); 0];
    
    % Setting control inputs to 0. 
    ux = 0;
    uy = 0;
    
    F = @(xBar, yBar, t) Dynamics3DSphericalPendulumOn3DCart(m, L, M, g, xBar, yBar, d, ux, uy);

    sOx = xBar0;
    sOy = yBar0;
    
    xm = zeros(1, length(t));
    ym = zeros(1, length(t));
    zm = zeros(1, length(t));
    
    Mx = zeros(1, length(t));
    My = zeros(1, length(t));
    theta = zeros(1, length(t));
    phi = zeros(1, length(t));
    
    for idx = 1:(length(t) - 1)
        [k_1x, k_1y] = F(sOx(:, idx), sOy(:, idx), t(idx));
        [k_2x, k_2y] = F(sOx(:, idx) + (k_1x / 2) * h, sOy(:, idx) + (k_1y / 2) * h, t(idx) + h / 2);
        [k_3x, k_3y] = F(sOx(:, idx) + (k_2x / 2) * h, sOy(:, idx) + (k_2y / 2) * h, t(idx) + h / 2);
        [k_4x, k_4y] = F(sOx(:, idx) + k_3x * h, sOy(:, idx) + k_3y * h, t(idx) + h);

        sOx(:, idx + 1) = sOx(:, idx) + (1/6) * (k_1x + (2 * k_2x) + (2 * k_3x) + k_4x) * h;
        sOy(:, idx + 1) = sOy(:, idx) + (1/6) * (k_1y + (2 * k_2y) + (2 * k_3y) + k_4y) * h;
    
        xm(idx) = sOx(1, idx) + L * sin(sOx(3, idx));
        ym(idx) = -L * cos(sOx(3, idx));
        zm(idx) = sOy(1, idx) + L * sin(sOy(3, idx));
    
        Mx(idx) = sOx(1, idx);
        My(idx) = sOy(1, idx);
    
        theta(idx) = sOx(3, idx) * 180 / pi;
        phi(idx) = sOy(3, idx) * 180 / pi;
    end

    if (ShowFig == 1)
        figure;
        hold on
        plot(t, Mx, 'b-');
        title("Cart x position");
        xlabel("time[s]");
        ylabel("Mx[t]");
        hold off

        figure;
        hold on
        plot(t, My, 'b-');
        title("Cart y position");
        xlabel("time[s]");
        ylabel("My[t]");
        hold off

        figure;
        hold on
        plot(t, theta, 'b-');
        title("theta position");
        xlabel("time[s]");
        ylabel("theta[t]");
        hold off

        figure;
        hold on
        plot(t, phi, 'b-');
        title("phi position");
        xlabel("time[s]");
        ylabel("phi[t]");
        hold off
    
        figure;
        hold on
        plot3(xm, zm, ym, 'r-');
        plot3(Mx, My, zeros(1, length(t)), 'b-');
        title("Pendulum Movement.");
        xlabel("x[m]");
        ylabel("y[m]");
        zlabel("z[m]");
        legend("Pendulum endpoint pos.", "Cart position");
        view(40, 40);
        hold off
    end
end