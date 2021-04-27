%   Simple example function that compares the results of estimating speed 
%   using a Kalman filter with noisy speed measurements, versus estimating 
%   speed using both noisy speed and acceleration measurements. 
function fig = test_kalman_filters_1D_2D

%%  First, a one-dimensional filter:
%
%   Create Noisy Data:
    t = linspace(0, 10, 1000);
    dt = t(2) - t(1);

    za = 2 * exp(-(t - 5).^2 / 2 * 4) / sqrt(2*pi/4);
    za = conv(za, ones(1, length(t)/2), 'same') * dt;
    za = [za, -za];
    zv = cumtrapz(za) * dt;

    t = linspace(0, t(end) * 2, length(t) * 2);
    
    za = za + randn(size(za)) * 1;
    zv = zv + randn(size(zv)) * 2;

%   Initialize Filter:
    xe = zeros(1, 1);
    Pe = zeros(1, 1);

    F = 1;
    Qa = dt;
    Q = F * Qa * F.';

    H = 1;
    R = 2^2;

    x = zeros(1, length(t));
    z = zv;
    
%   Filter Data:
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
    
%%  Second, a two-dimensional filter:
% 
%   Initialize Filter:
    xe = zeros(2, 1);
    Pe = zeros(2, 2);
    
    F = [1, dt; 0, 1];
    Qa = [0, 0; 0, 1/dt];
    Q = F * Qa * F.';
    
    H = eye(2);
    R = [2^2, 0; 0, 1];
    
    x = zeros(2, length(t));
    z = [zv; za];
    
%   Filter Data:
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

        %% KALMAN_FILTER  Standard single iteration of a Kalman filter. 
        %  
        %    [xe, Pe] = KALMAN_FILTER(xe, Pe, z) takes in previous state 
        %    estimates, xe, their covariances, Pe, and current measurements 
        %    to return updated current estimates and covariances. 
        % 
        %    Other parameters F, Q, H, and R are assumed to be global 
        %    variables for this example implementation. 
        function [xe, Pe] = kalman_filter(xe, Pe, z)
        %   1) Predict:
            xp = F * xe;
            Pp = F * Pe * F.' + Q;
            
        %   2) Weigh:  
            K = Pp * H.' * ((H * Pp * H.' + R) \ eye(size(R)));
            
        %   3) Estimate:  
            xe = xp + K * (z - H * xp);
            Pe = (eye(size(Pp)) - K * H) * Pp * (eye(size(Pp)) - K * H).' + K * R * K.';
        end
end
