%GETANALYTIC   Computes the complex, analytic representation of a
%   function, as well as its instantaneous frequency.
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

isodd = mod(length(y), 2) > 0;
if isodd
    Y = fft([y; y(end)]);
else
    Y = fft(y);
end
n = ceil(length(Y) / 2);

ya = fft([Y(1); 2*Y(2:n-1); Y(n); zeros(n, 1)]);

if isodd
    ya = ya(1:end-1);
end

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
