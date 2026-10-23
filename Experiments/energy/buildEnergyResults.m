function mlx = buildEnergyResults()
% Rebuilds EnergyResults.mlx (repository root) from its plain-text source,
% Experiments/energy/energyResultsSource.m. Edit the source, then run this; the live script is a
% binary file, so the source is what version control can diff.
%   matlab -batch "addpath(genpath(pwd)); buildEnergyResults"
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
mlx = fullfile(root, 'EnergyResults.mlx');
if isfile(mlx), delete(mlx); end
matlab.internal.liveeditor.openAndSave(fullfile(here, 'energyResultsSource.m'), mlx);
fprintf('Built %s\n', mlx);
end
