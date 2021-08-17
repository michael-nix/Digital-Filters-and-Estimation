%GETANALYTIC   Computes the complex, analytic representation of a
%   function, as well as its instantaneous frequency, by first finding its
%   Hilbert transform.
%
%   ya = GETANALYTIC(y, x) returns the analytic representation of y, a
%   function of x.
%
%   [ya, fi] = GETANALYTIC(y, x) also returns the instantaneous frequency,
%   fi, which is just the gradient of the angle of ya.
%
%   See also: GETEMD, EMDFILTER.
function [ya, fi] = getanalytic(y, x)

wasrow = false;
if isrow(y)
    wasrow = true;
    y = y.';
end

Y = fft(y);
n = round(length(Y) / 2);

ya = y + ifft([-Y(1:n); Y(n+1:end)]);

if nargout > 1
    phi = unwrap(angle(ya)) / 2 / pi;
    fi = gradient(phi, x);
    if wasrow
        fi = fi.';
    end
end

if wasrow
    ya = ya';
end