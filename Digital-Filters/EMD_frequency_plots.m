clear;
load EMDFilterData;

[f, c, r] = emdfilter(y, x, 2);

[x1, y1, idx] = mirrordata(x, c(:,1));
[x3, y3] = mirrordata(x, c(:,3));
[x5, y5] = mirrordata(x, c(:,5));

[~, fi1] = getanalytic(y1, x1);
[~, fi3] = getanalytic(y3, x3);
[~, fi5] = getanalytic(y5, x5);
edges = 0:0.125:16;

figure;
histogram(abs(fi1(idx)), edges, 'Normalization', 'Probability'); hold on;
histogram(abs(fi3(idx)), edges, 'Normalization', 'Probability');
histogram(abs(fi5(idx)), edges, 'Normalization', 'Probability'); grid on;
xlabel('Frequency (Hz)'); ylabel('Probability (%)');
title('Instantaneous Frequency Distribution of IMFs');
axis tight; xlim([0, 16]);

p1 = abs(fft(c(:,1))); p1 = p1(1:end/2); p1 = p1 / sum(p1);
p3 = abs(fft(c(:,3))); p3 = p3(1:end/2); p3 = p3 / sum(p3);
p5 = abs(fft(c(:,5))); p5 = p5(1:end/2); p5 = p5 / sum(p5);
f0 = linspace(0, 16, length(x)/2).';

figure; 
plot(f0, p1, f0, p3, f0, p5, 'LineWidth', 1.5); grid on;
xlabel('Frequency (Hz)'); ylabel('Probability (%)');
title('Fourier Transform Frequency Distributions of IMFs');
