%GETIMFS   Performs Empirical Mode Decomposition (EMD) on data in order to
%   compute and return the Intrinsic Mode Functions (IMFs).  Sifting is 
%   based on standard deviation metric in the original by Huang et al, but 
%   uses shortcuts on monotonicity and peak finding to speed things up.
%
%   imf = GETIMFS(y, x) returns the Intrinsic Mode Functions of y, a
%   function of x, sifting a maximum of 100 times to find each IMF.
%
%   imf = GETIMFS(..., maxnimf) returns up to maxnimf IMFs regardless of
%   how many there actually may be.  However, the EMD process will stop
%   whenever the residual becomes approximately monotonic.
%
%   imf = GETIMFS(..., idx) uses an index vector given by idx to find the
%   standard deviation of your IMF in the sifting process.  By default the
%   full y vector is used, but to eliminate end effects, you might want to
%   focus only on a specific region of interest, i.e. defined by idx.
%
%   [imf, r] = getimfs(y, x) also returns the residual, r.
%
%   [imf, r, nimf] = getimfs(y, x) also returns the total number of IMFs
%   found.
%
%   See also: EMDFILTER, GETANALYTIC.
function [imf, r, nimf] = getimfs(y, x, maxnimf, idx)
    
    useidx = false;
    if nargin == 4
        useidx = true;
    end

    wasrow = false;
    if isrow(y)
        wasrow = true;
        y = y(:);
        x = x(:);
    end

    imf = zeros(length(y), maxnimf);
    for i = 1:maxnimf
        h = y;
        
        extrema = conv([1, -1], sign(diff(h))); 
        idxmax = extrema == -2; 
        idxmin = extrema == 2;
        
        maybemonotonic = (length(find(idxmax, 3)) < 3) || (length(find(idxmin, 3)) < 3);
        if maybemonotonic
            i = i - 1; %#ok<FXSET>
            break;
        end
        
        upper = spline(x(idxmax), h(idxmax), x);
        lower = spline(x(idxmin), h(idxmin), x);

        meanenv = (upper + lower) / 2;
        h = h - meanenv;
        
        for j = 1:100
            extrema = conv([1, -1], sign(diff(h)));
            idxmax = extrema == -2;
            idxmin = extrema == 2;
            
            maybemonotonic = (length(find(idxmax, 3)) < 3) || (length(find(idxmin, 3)) < 3);
            if maybemonotonic
                break;
            end
            
            upper = spline(x(idxmax), h(idxmax), x);
            lower = spline(x(idxmin), h(idxmin), x);

            meanenv = (upper + lower) / 2;
            
            if useidx
                SD = sum(meanenv(idx).^2 ./ h(idx).^2);
            else
                SD = sum(meanenv.^2 ./ h.^2);
            end

            h = h - meanenv;
            if SD < 0.3
                break;
            end
        end

        imf(:,i) = h;
        y = y - h;
    end
    r = y;
    nimf = i;
    imf = imf(:,1:nimf);
    
    if wasrow
        imf = imf.';
        r = r.';
    end
end