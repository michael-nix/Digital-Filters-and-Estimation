function filtered = averaging_emdfilter(f, t, nsamples, fc)

if nargin < 3
    % 86 samples gives a ROI size of 256, 342 is 1024.
    nsamples = 86;
    fc = 0.5;
end

nsegments = floor(length(t) / nsamples);
filtered = zeros(length(t), 4);

for i = 0:(nsegments - 4)
    idx = (nsamples*i + 1):(nsamples*(i + 1));    
    filtered(idx, 1) = emdfilter(f(idx), t(idx), fc);
    
    idx = idx + round(nsamples / 4);
    filtered(idx, 2) = emdfilter(f(idx), t(idx), fc);
    
    idx = idx + round(nsamples / 2);
    filtered(idx, 3) = emdfilter(f(idx), t(idx), fc);
     
    idx = idx + round(nsamples * 3 / 4);
    filtered(idx, 4) = emdfilter(f(idx), t(idx), fc);
end

filtered = sum(filtered, 2) / 4;
