function [imf, res, nimf] = getemd(y, x, maxnimf)

if nargin < 3
    maxnimf = 10;
end

if isrow(x)
    x = x.';
end

wasrow = false;
if isrow(y)
    wasrow = true;
    y = y.';
end

[x0, y0, idxROI] = mirrordata(x, y);

[imf, res, nimf] = getimfs(y0, x0, maxnimf, idxROI);

imf = imf(idxROI,1:nimf);
if wasrow
    imf = imf.';
end

if nargout > 1
    res = res(idxROI);
    if wasrow
        res = res.';
    end
end