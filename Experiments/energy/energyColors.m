function [C, names] = energyColors(method)
% One fixed color per method across the energy paper's figures (rows of C, in the order of
% names); with a method name, its color only
names = ["EA-SO", "EA-FO-3", "EA-FO-0", "BA", "ED-RHC", "ED-RHC-alpha", "ED-ORHC-SO", "ED-ORHC-FO-3"];
C = [0.00 0.45 0.74; 0.47 0.67 0.19; 0.30 0.75 0.93; 0.85 0.33 0.10; ...
     0.49 0.18 0.56; 0.64 0.08 0.18; 0.93 0.69 0.13; 0.40 0.40 0.40];
if nargin > 0
    k = find(names == method, 1);
    if isempty(k), k = 1; end
    C = C(k, :);
end
end
