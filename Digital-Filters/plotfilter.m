%PLOTFILTER  Plots the frequency response of a digital filter.
%
%   PLOTFILTER(alpha, beta) plots the filter given the coefficients of the
%   expanded polynomial for the denominator (alpha) and numerator (beta) of
%   the Z-transform of a digital filter.
%
%   PLOTFILTER(alpha, beta, n) plots the same filter, but with n data
%   points; default n is 1024.
%
%   [H, w] = PLOTFILTER(alpha, beta, n) does not plot the filter, but
%   returns the transfer function, H, and the frequency values (w) along
%   the unit circle that were used to generate H.
%
%   See also GETNOTCH, GETBUTTER.
function varargout = plotfilter(alpha, beta, n)
    
    if nargin < 3
        n = 2^10;
    end
    
    w = linspace(-pi, pi, n);
    zi = exp(-1i*w);
    
    num = beta(1) * ones(size(zi));
    for i = 2:length(beta)
        num = num + beta(i) * zi.^(i-1);
    end
    
    den = alpha(1) * ones(size(zi));
    for i = 2:length(alpha)
        den = den + alpha(i) * zi.^(i-1);
    end
    
    H = num ./ den;
    
    if nargout == 0
        figure;
        plot(w/2/pi, abs(H), 'LineWidth', 1.5);
        grid on;
        
        axis([-0.5 0.5 0 1.05]);
        xlabel('Normalized Frequency (Hz)');
        ylabel('|H(e^{j\omega})|');
    else
        varargout{1} = H;
        varargout{2} = w;
    end
end
