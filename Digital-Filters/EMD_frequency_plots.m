clear;
load EMDFilterData;

%%
[c, r] = getemd(y, x);

[x1, y1, idx] = mirrordata(x, c(:,1));
[x3, y3] = mirrordata(x, c(:,3));
[x5, y5] = mirrordata(x, c(:,5));

[ya1, fi1] = getanalytic(y1, x1);
[ya3, fi3] = getanalytic(y3, x3);
[ya5, fi5] = getanalytic(y5, x5);
edges = 0:0.125:16;

ya1 = abs(ya1(idx)) / sum(abs(ya1(idx))); fi1 = fi1(idx);
ya3 = abs(ya3(idx)) / sum(abs(ya3(idx))); fi3 = fi3(idx);
ya5 = abs(ya5(idx)) / sum(abs(ya5(idx))); fi5 = fi5(idx);

%%
p1 = abs(fft(c(:,1))); p1 = p1(1:end/2); p1 = p1 / sum(p1);
p3 = abs(fft(c(:,3))); p3 = p3(1:end/2); p3 = p3 / sum(p3);
p5 = abs(fft(c(:,5))); p5 = p5(1:end/2); p5 = p5 / sum(p5);
f0 = linspace(0, 16, length(x)/2).';

%%
figure;
plot(fi1, ya1, 'o', fi3, ya3, 'o', fi5, ya5, 'o', ...
    'LineWidth', 1.5);
xlabel('Frequency (Hz)'); ylabel('Relative Energy');
title('Marginal Hilbert Spectrum of IMFs');
grid on; xlim([0, 16]);
legend('IMF 1', 'IMF 3', 'IMF 5');

%%
figure;
histogram(abs(fi1), edges, 'Normalization', 'Probability'); hold on;
histogram(abs(fi3), edges, 'Normalization', 'Probability');
histogram(abs(fi5), edges, 'Normalization', 'Probability'); grid on;
xlabel('Frequency (Hz)'); ylabel('Relative Energy');
title('Instantaneous Frequency Distribution of IMFs');
axis tight; xlim([0, 16]);
legend('IMF 1', 'IMF 3', 'IMF 5');

%%
figure; 
plot(f0, p1, f0, p3, f0, p5, 'LineWidth', 1.5); grid on;
xlabel('Frequency (Hz)'); ylabel('Relative Energy');
title('Normalized Fourier Transform of IMFs');
legend('IMF 1', 'IMF 3', 'IMF 5');