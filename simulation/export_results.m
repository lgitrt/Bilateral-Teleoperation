function export_results(destination)
%EXPORT_RESULTS Reproduce lightweight simulation figures for documentation.
arguments
    destination (1,1) string = fullfile(fileparts(mfilename("fullpath")), "..", "docs")
end
if ~isfolder(destination)
    mkdir(destination);
end
out = run_model("FourCh_TDPA_TD", 10);
fig = figure("Visible", "off", "Color", "white", "Position", [100 100 1000 600]);
cleanup = onCleanup(@() close(fig));
tiledlayout(2,1);
nexttile;
draw_signal(out, "xm", "Master");
hold on;
draw_signal(out, "xs", "Slave");
grid on; legend("Location", "best");
xlabel("Time [s]"); ylabel("Position [m]");
title("Four-channel reference simulation: master / slave position");
nexttile;
draw_signal(out, "fm", "Master control");
hold on;
draw_signal(out, "fs", "Slave control");
grid on; legend("Location", "best");
xlabel("Time [s]"); ylabel("Force [N]");
title("Controller output forces (simulation, not hardware measurements)");
exportgraphics(fig, fullfile(destination, "simulation-response.png"), "Resolution", 140);
end

function draw_signal(out, name, label)
element = out.logsout.getElement(name);
values = element.Values;
assert(all(isfinite(values.Data(:))), "teleop:NonFinitePlot", "Non-finite plot data.");
plot(values.Time, values.Data, "DisplayName", label, "LineWidth", 1.3);
end
