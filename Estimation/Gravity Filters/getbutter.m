%GETBUTTER  Find expanded polynomial coefficients for Butterworth filters.
%
%   [alpha, beta] = GETBUTTER(f0, dt, m) returns the expanded polynomial
%   cofficients for a digital Butterworth filter with cutoff frequency, f0, 
%   sampling interval, dt (inverse of sample rate), and order, m 
%   (polynomial degree).  Alpha is a vector containing the polynomial 
%   coefficients of the denominator of the transfer function of the
%   Butterworth filter, and beta is a vector containing the polynomial
%   coefficients of the numerator.
%
%   To implement a filter with alpha and beta, you can use something like:
%   
%   y(n) = sum(x(n:-1:n-m-1).*beta) - sum(y(n-1:-1:n-m-1).*alpha(2:end));
%
%   Where x(n) is your input signal and y(n) is your output signal.
%
%   See also GETNOTCH, PLOTFILTER, FILTER.
function [alpha, beta] = getbutter(f0, dt, m)
    if nargin < 2
        dt = 1;
    end
    
    if nargin < 3
        m = 3;
    end
    
    p = exp(1i * (2*(1:m).' + m - 1) * pi / 2 / m);
    
    w0 = 2*pi*dt*f0;
    w0 = 2/dt * tan(w0 * dt/2);
    
    alpha = [(1 - w0/2 * p(1)); -(1 + w0/2 * p(1))];
    for i = 2:m
        alpha = conv(alpha, [(1 - w0/2 * p(i)); -(1 + w0/2 * p(i))]);
    end
    alpha = real(alpha);
    
    beta = w0/2 * [1; 1];
    for i = 2:m
        beta = conv(beta, w0/2*[1; 1]);
    end
    
    beta = beta / alpha(1);
    alpha = alpha / alpha(1);
end