% mirror data so region of interest (ROI) is in the middle, removing end
% effects in each IMF from the EMD process:
function [x0, y0, idxROI] = mirrordata(x, y)

idxROI = (1:length(x)) + length(x) - 1;

x0 = [-x(end:-1:2) + 2*x(1); x; ...
      -x(end-1:-1:1) + 2*x(end)];

y0 = [y(end:-1:2); y; ...
      y(end-1:-1:1)];
