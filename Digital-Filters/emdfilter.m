%EMDFILTER   Using Empirical Mode Decomposition (EMD), filter a signal by
%   eliminating Intrinsic Mode Functions (IMFs) whose mean instantaneous
%   frequency is outside the area of interest.  The input signal is
%   mirrored on either side in order to eliminate the end effects inherent
%   in the EMD process.
%
%   f = EMDFILTER(y, x) returns the filtered signal, f, based on the input
%   signal, y, a function of x.  With no upper or lower cutoff frequencies
%   given, the defaults are a low-pass filter removing 50% of lower
%   frequency content.
%
%   f = EMDFILTER(..., fup, flo) returns the filtered signal using an upper
%   cutoff frequency of fup, and a lower cutoff frequency of flo.  All
%   units for frequency are in Hertz.
%
%   f = EMDFILTER(..., maxnimf) uses a maximum of maxnimf IMFs to filter.
%
%   [f, imf, res] = EMDFILTER(...) also returns the IMFs that were computed
%   by the EMD, imf, as well as the residual signal, res.
%
%   See also: GETIMFS, GETANALYTIC.
function [f, imf, res] = emdfilter(y, x, fup, flo, maxnimf)

narginchk(2, 5);
if nargin < 5
    maxnimf = 10;
end

% these defaults set up a low-pass filter for bottom 50% of frequencies:
if nargin < 4
    flo = 0;
end

if nargin < 3
    maxf = 1 / min(diff(x)) / 2;
    fup = maxf / 2;
end

if isrow(x)
    x = x.';
end

wasrow = false;
if isrow(y)
    wasrow = true;
    y = y.';
end

% mirror data so region of interest (ROI) is in the middle, eliminating end
% effects in each IMF from the EMD process:
idxROI = (1:length(x)) + length(x) - 1;
x0 = [-x(end:-1:2) + 2*x(1); x; -x(end-1:-1:1)+2*x(end)];
y0 = [y(end:-1:2); y; y(end-1:-1:1)];

[imf, res, nimf] = getimfs(y0, x0, maxnimf, idxROI);

f = zeros(length(x), 1);
for i = nimf:-1:1
    [~, phi] = getanalytic(imf(:, i), x0);
    
    maxfreq = mean(abs(phi(idxROI)));    
    if (maxfreq > fup)
        break;
    elseif (maxfreq < flo)
        continue;
    end
    f = f + imf(idxROI, i);
end

% do a quick check to see if the residual has useful signal:
[~, phi] = getanalytic(res, x0);

maxfreq = mean(abs(phi(idxROI)));
if (maxfreq < fup) && (maxfreq > flo)
    f = f + res(idxROI);
end

if wasrow
    f = f.';
end

if nargout > 1
    imf = imf(idxROI,1:nimf);
    if wasrow
        imf = imf.';
    end
end

if nargout > 2
    res = res(idxROI);
    if wasrow
        res = res.';
    end
end