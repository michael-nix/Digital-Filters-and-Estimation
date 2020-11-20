clear; close all;

w = linspace(-pi, pi, 2^12);

zi = exp(-1i*w);

f0 = 0.05;
dt = 1;
w0 = 2 * pi * f0 * dt;

b = @(w) sqrt( 12 ./ w.^2 * (1 - cos(w)) ./ (1 + cos(w)));

H = (1 - zi) ./ ((1 + b(w0)*w0/2) - (1 - b(w0)*w0/2) * zi);

figure;

subplot(2, 1, 1);
plot(w / 2 / pi, abs(H), 'LineWidth', 1.5);
title('Magnitude and Phase of Transfer Function H(z)');
ylabel('|H(e^{j\omega})|');
grid on;

subplot(2, 1, 2);
plot(w / 2 / pi, angle(H), 'LineWidth', 1.5);
ylabel('\angleH(e^{j\omega})');
xlabel('Normalized Frequency');
grid on;