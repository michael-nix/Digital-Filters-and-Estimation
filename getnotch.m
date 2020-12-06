%GETNOTCH  Find expanded polynomial coefficients for digital notch filters.
%
%   [alpha, beta] = GETNOTCH(f0, dt, m) returns the expanded polynomial
%   cofficients for a digital notch filter with notch frequency of 0 Hz,
%   cutoff frequency, f0, sampling interval, dt (inverse of sample rate),
%   and order, m (polynomial degree).  Alpha is a vector containing the
%   polynomial coefficients of the denominator of the transfer function of
%   the notch filter, and beta is a vector containing the polynomial
%   coefficients of the numerator.
%
%   [alpha, beta] = GETNOTCH([fn, bw], dt, m) returns the same as before,
%   but with a notch frequency of fn Hz, and a HWHM bandwidth of bw Hz.  In
%   order to return real coefficients, the order of the filter, m, is
%   doubled.
%
%   To implement a filter with alpha and beta, you can use something like:
%   
%   y(n) = sum(x(n:-1:n-m-1).*beta) - sum(y(n-1:-1:n-m-1).*alpha(2:end));
%
%   Where x(n) is your input signal and y(n) is your output signal.
%
%   See also GETBUTTER, PLOTFILTER, FILTER.
function [alpha, beta] = getnotch(f0, dt, m)

    if nargin < 2
        dt = 1;
    end
    
    if nargin < 3
        m = 1;
    end
    
    if isscalar(f0)        
        w0 = 2*pi*f0*dt;
        
        alpha = [(1 + w0/2); -(1 - w0/2)];
        for i = 2:m
            alpha = conv(alpha, [(1 + w0/2); -(1 - w0/2)]);
        end
        alpha = real(alpha);
        
        beta = [1; -1];
        for i = 2:m
            beta = conv(beta, [1; -1]);
        end
    else
        fn = f0(1);   % notch frequency; the one that needs to be cut out.
        bw = f0(2);   % approx. HWHM power bandwidth at notch frequency.
                
        w0 = 2*pi*bw*dt;
        p1 = exp(1i*2*pi*fn*dt);
        p2 = conj(p1);
        
        alpha = conv([(1 + w0/2); -(1 - w0/2) * p1], ...
                     [(1 + w0/2); -(1 - w0/2) * p2]);
        for i = 2:m
            alpha = conv(alpha, [(1 + w0/2); -(1 - w0/2) * p1]);
            alpha = conv(alpha, [(1 + w0/2); -(1 - w0/2) * p2]);
        end
        alpha = real(alpha);
        
        beta = conv([1; -p1], [1; -p2]);
        for i = 2:m
            beta = conv(beta, [1; -p1]);
            beta = conv(beta, [1; -p2]);
        end
        beta = real(beta);
    end
    
    beta = beta / alpha(1);
    alpha = alpha / alpha(1);
end
