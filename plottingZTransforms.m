clear; close all; 

% we need to evaluate the Z-Transform around the unit circle, and there are
% only 2 pi angles we get to choose from, so we set up our domain as: 
w = linspace(-pi, pi, 2^12); 

% to evaluate along the unit circle in the complex domain, we need our z to
% be set to complex numbers representing the unit circle.  Or rather, we 
% set our inverse, zi, to be the inverse of that: 
zi = exp(-1i*w); 

% these are just the initial parameters: 
f0 = 0.05; 
dt = 1; 
w0 = 2 * pi * f0 * dt; 

% instead of defining a separate function somewhere, we can set up our 
% adjustment factor, b, as an inline, or anonymous, function: 
b = @(w) sqrt( 12 ./ w.^2 * (1 - cos(w)) ./ (1 + cos(w))); 

% this is just the transfer function itself, exactly as written down.  The 
% only  caveat is that the inverse z, zi, we defined above to be points 
% along the unit circle: 
H = (1 - zi) ./ ((1 + b(w0)*w0/2) - (1 - b(w0)*w0/2) * zi); 

% now we just gotta plot everything: 
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
