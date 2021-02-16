function fig = test_kalman_filters

    t = linspace(0, 10, 1000);
    dt = t(2) - t(1);

    za = 2 * exp(-(t - 5).^2 / 2 * 4) / sqrt(2*pi/4);
    za = conv(za, ones(1, length(t)/2), 'same') * dt;
    za = [za, -za];
    zv = cumtrapz(za) * dt;

    za = za + randn(size(za)) * 1;
    zv = zv + randn(size(zv)) * 2;

    t = linspace(0, t(end) * 2, length(t) * 2);

    xe = zeros(1, 1);
    Pe = zeros(1, 1);

    F = 1;
    Qa = dt;
    Q = F * Qa * F.';

    H = 1;
    R = 2^2;

    x = zeros(1, length(t));
    z = zv;
    for i = 1:length(t)

        [xe, Pe] = kalman_filter(xe, Pe, z(:,i));

        x(:,i) = xe;
    end

    fig = figure;
    subplot(1, 2, 1);
    plot(t, zv); grid on; hold on;
    plot(t, x, 'LineWidth', 1.5);
    title('One-Dimensional Filter');
    axis([0 20 -5 15]);
    
    xe = zeros(2, 1);
    Pe = zeros(2, 2);
    
    F = [1, dt; 0, 1];
    Qa = [0, 0; 0, 1/dt];
    Q = F * Qa * F.';
    
    H = eye(2);
    R = [2^2, 0; 0, 1];
    
    x = zeros(2, length(t));
    z = [zv; za];
    
    for i = 1:length(t)
        
        [xe, Pe] = kalman_filter(xe, Pe, z(:,i));
        
        x(:,i) = xe;
    end
    
    figure(fig);
    subplot(1, 2, 2);
    plot(t, zv); grid on; hold on;
    plot(t, x(1,:), 'LineWidth', 1.5);
    title('Two-Dimensional Filter');
    axis([0 20 -5 15]);
    legend('measurement', 'estimate', 'Location', 'South');
    
    ax = axes(fig, 'visible', 'off');
    ax.XLabel.Visible = 'on';
    ax.YLabel.Visible = 'on';
    ylabel(ax, 'speed (m/s)');
    xlabel(ax, 'time (s)');

        function [x0, P0] = kalman_filter(x0, P0, z)
            xp = F * x0;
            Pp = F * P0 * F.' + Q;

            K = Pp * H.' * ((H * Pp * H.' + R) \ eye(size(R)));
            
            x0 = xp + K * (z - H * xp);
            P0 = (eye(size(Pp)) - K * H) * Pp * (eye(size(Pp)) - K * H).' + K * R * K.';
        end
end