function energyFigures(outDir)
% Figures of the energy-aware paper that need no simulation, written as vector PDFs (and
% 300-dpi PNGs) to outDir (default: results/energy/):
%   fig_profiles.pdf  S1: (a) SO speed profiles with c2 = 0 and with c2 > 0 at three transit
%       times, (b) the motion loss against the transit time for the least-loss SO profile
%       (Theorem 1), the c2 = 0 optimum under the full model, and FO-3 (Theorem 3), (c) FO-3's
%       acceleration time tau*/rho; the energy robots' coefficients (c1, c2) = (17.75, 1.16)
%   fig_setups.pdf    the energy setups (Configurations/Energy) as the GUI draws them, with their
%       stations and docks and the agents' start targets
arguments
    outDir (1,:) char = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results', 'energy')
end
if ~isfolder(outDir), mkdir(outDir); end
profilesFigure(outDir);
setupsFigure(outDir);
end

function profilesFigure(outDir)
c1 = 17.75; c2 = 1.16; nu = sqrt(c2 / c1); y = 20;
rhos = [10 20 40];
cols = energyColors();
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 7.16 2.2]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
% (a) speed profiles, normalized by the mean speed y/rho
nexttile; hold on; grid on; box on;
x = linspace(0, 1, 201);
P0 = SpeedProfile.shape("SO", y, rhos(1));
plot(x, speedAt(P0, x) / (y / rhos(1)), 'k--', 'LineWidth', 1.1, 'DisplayName', '$c_2 = 0$');
for k = 1:numel(rhos)
    P = SpeedProfile.shape("SO", y, rhos(k), Inf, Inf, nu);
    plot(x, speedAt(P, x) / (y / rhos(k)), 'LineWidth', 1.1, 'Color', cols(k, :), ...
        'DisplayName', sprintf('$\\rho = %d$ s', rhos(k)));
end
xlabel({'Normalized time: $t/\rho$', '(a) SO profiles'}, 'Interpreter', 'latex');
ylabel('Normalized speed: $v(t)\,\rho/y$', 'Interpreter', 'latex');
legend('Interpreter', 'latex', 'Location', 'south', 'FontSize', 8, 'Box', 'on');
style(gca);
% (b) motion loss against rho
nexttile; hold on; grid on; box on;
r = linspace(8, 60, 120);
Lso = SpeedProfile.loss("SO", y, r, Inf, Inf, c1, c2);
Lcub = arrayfun(@(q) fullLoss(SpeedProfile.shape("SO", y, q), c1, c2), r);
Lfo3 = SpeedProfile.loss("FO-3", y, r, Inf, Inf, c1, c2);
plot(r, Lso, 'LineWidth', 1.1, 'Color', cols(1, :), 'DisplayName', 'SO (Thm. 1)');
plot(r, Lcub, '--', 'LineWidth', 1.1, 'Color', cols(2, :), 'DisplayName', '$c_2 = 0$ optimum');
plot(r, Lfo3, 'LineWidth', 1.1, 'Color', cols(3, :), 'DisplayName', 'FO-3 (Thm. 3)');
set(gca, 'YScale', 'log');
xlabel({'Transit time: $\rho$ [s]', '(b) Motion loss'}, 'Interpreter', 'latex');
ylabel('Motion loss: $L$ [J]', 'Interpreter', 'latex');
legend('Interpreter', 'latex', 'Location', 'northeast', 'FontSize', 8, 'Box', 'on');
style(gca);
% (c) FO-3's acceleration time
nexttile; hold on; grid on; box on;
tau = arrayfun(@(q) fo3Tau(y, q, nu), r);
plot(r, tau ./ r, 'LineWidth', 1.1, 'Color', cols(3, :), 'DisplayName', '$c_2 > 0$');
plot(r, ones(size(r)) / 3, 'k--', 'LineWidth', 1.1, 'DisplayName', '$c_2 = 0$');
xlabel({'Transit time: $\rho$ [s]', '(c) FO-3 acceleration time'}, 'Interpreter', 'latex');
ylabel('Acceleration share: $\tau^\ast/\rho$', 'Interpreter', 'latex');
legend('Interpreter', 'latex', 'Location', 'southwest', 'FontSize', 8, 'Box', 'on');
ylim([0 0.4]);
style(gca);
saveFig(f, fullfile(outDir, "fig_profiles"));
end

function v = speedAt(P, x)
v = zeros(size(x));
for k = 1:numel(x), [~, v(k)] = SpeedProfile.at(P, x(k) * P.rho); end
end

function L = fullLoss(P, c1, c2)
[Eu, Ev] = SpeedProfile.integrals(P);
L = c1 * Eu + c2 * Ev;
end

function t = fo3Tau(y, rho, nu)
P = SpeedProfile.shape("FO-3", y, rho, Inf, Inf, nu);
t = P.s;
end

function setupsFigure(outDir)
% The energy setups as the GUI draws them: background, walls, the paths of the edges (Dijkstra
% Corners), targets, stations (squares, with their docks) and the agents' start targets
root = fileparts(which('MainUI'));
lays = ["Energy_Office_1Dock", "Energy_Campus_2Stations"];
names = ["Office: one station, one dock", "Campus: two stations, two docks each"];
f = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 7.16 3.4]);
tiledlayout(1, numel(lays), 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:numel(lays)
    s = load(fullfile(root, 'Configurations', 'Energy', lays(k) + ".mat"), 's').s;
    model = ScenarioModel(DijkstraCornerPlanner());
    model.importLayout(s);
    ax = nexttile; hold on; axis equal; box on;
    if isfield(s, 'background') && s.background ~= ""
        img = flipud(imread(fullfile(root, 'Configurations', 'backgrounds', s.background)));
        image(ax, [0 100], [0 100], img, 'AlphaData', 0.85);
    end
    for w = 1:size(s.walls, 1)
        plot(s.walls(w, [1 3]), s.walls(w, [2 4]), '-', 'Color', [0.2 0.2 0.2], 'LineWidth', 1);
    end
    for e = 1:numel(model.edges)
        c = model.edges(e).curvePoints;
        plot(c(:, 1), c(:, 2), '-', 'Color', [0.1 0.1 0.1], 'LineWidth', 0.6);
    end
    for q = 1:size(s.targets, 1)
        st = find(s.chargers == q, 1);
        if isempty(st)
            plot(s.targets(q, 1), s.targets(q, 2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.75 0.85 1.0], ...
                'MarkerEdgeColor', [0.1 0.2 0.5]);
            text(s.targets(q, 1), s.targets(q, 2), sprintf('%d', q), 'FontSize', 6.5, ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Interpreter', 'latex');
        else   % a station (its docks are in the title)
            plot(s.targets(q, 1), s.targets(q, 2), 's', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.82 0.1], ...
                'MarkerEdgeColor', [0.4 0.3 0]);
        end
    end
    for a = s.agentStarts(:)'
        startMarker(s.targets(a, :), 2.0, 3.8);
    end
    set(ax, 'YDir', 'normal', 'XTick', [], 'YTick', [], 'XLim', [0 100], 'YLim', [0 100]);
    xlabel(sprintf('(%s) %s ($M = %d$, $N = %d$)', char('a' + k - 1), names(k), size(s.targets, 1) - numel(s.chargers), ...
        numel(s.agentStarts)), 'Interpreter', 'latex', 'FontSize', 8);   % a subcaption, not a title
end
saveFig(f, fullfile(outDir, "fig_setups"));
end

function startMarker(p, r, gap)
% an agent's start: a red triangle above its target (y up)
patch(p(1) + r * [-1 1 0], p(2) + gap + r * [-0.8 -0.8 1], [0.85 0.1 0.1], 'EdgeColor', 'k', 'LineWidth', 0.4);
end

function style(ax)
set(ax, 'FontSize', 8, 'TickLabelInterpreter', 'latex');
end

function saveFig(f, base)
exportgraphics(f, base + ".pdf", 'ContentType', 'vector');
exportgraphics(f, base + ".png", 'Resolution', 300);
close(f);
end
