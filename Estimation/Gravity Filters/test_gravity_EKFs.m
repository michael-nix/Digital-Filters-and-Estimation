clear;

% contains ax, ay, az, acctime, gpsspeed, gpstime:
load('EKFGravityTestData.mat');

clampingfilter = false;
gravityfilter = ~clampingfilter;

%#ok<*UNRCH>
if clampingfilter
    nstate = 6; 
elseif gravityfilter
    nstate = 9;
end
x0 = zeros(nstate, 1);
P0 = eye(nstate) * 10;
x_est = zeros(length(x0), length(acctime));

gpsidx = 1;
accidx = 1;
accelerometer_event = 1;
gps_event = 2;

v = 0;
a = [ax(1); ay(1); az(1)];
dt = acctime(2) - acctime(1);
fs = 1 / mean(diff(acctime));

m = 3;
[alpha, beta] = getbutter(0.01, 1 / fs, m);
alpha = alpha.';
beta = beta.';
a_raw = zeros(3, m+1);
a_filtered = zeros(3, m);
g = zeros(3, 1);

while true
    [~, sensor_event] = ...
        min([acctime(accidx), gpstime(gpsidx)]);
    
    switch sensor_event
        case accelerometer_event
            a_now = [ax(accidx); ay(accidx); az(accidx)];
            
            if clampingfilter
                a_raw = [a_now, a_raw(:,1:end-1)];
                g = sum(a_raw .* beta, 2) - sum(a_filtered .* alpha(2:end), 2);
                a_filtered = [g, a_filtered(:,1:end-1)];
                a_now = a_now - g;
                
                z = [v; a_now];
                [x0, P0] = ekf_va(z, dt, x0, P0);
                x_est(:, accidx) = x0;

            elseif gravityfilter
                z = [v; 9.80665; a_now];
                [x0, P0] = ekf_vga(z, dt, x0, P0);
                x_est(:, accidx) = x0;
            end
            
            accidx = accidx + 1;
            dt = acctime(accidx) - acctime(accidx - 1);
            
        case gps_event
            v = gpsspeed(gpsidx);
            gpsidx = gpsidx + 1;
            
    end
    
    if gpsidx == length(gpstime) || accidx == length(acctime)
        break;
    end
end

results_fig = figure;
if clampingfilter
    plot(acctime, ay, acctime, x_est(5,:), 'LineWidth', 1.5);

elseif gravityfilter
    plot(acctime, ay, acctime, x_est(5,:) - x_est(8,:), 'LineWidth', 1.5);
end
axis([450 540 -3 4]); grid on;
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
legend('raw data', 'filtered');
results_fig.Position = [200 500 750 420];
