function destination = export_animation(destination)
%EXPORT_ANIMATION Animate actual four-channel Simulink motion and forces.
arguments
    destination (1,1) string = fullfile(fileparts(mfilename("fullpath")), ...
        "..", "docs", "simulation-animation.gif")
end
folder = fileparts(destination);
if ~isfolder(folder)
    mkdir(folder);
end
out = run_model("FourCh_TDPA_TD", 10);
names = ["xm", "xs", "fm", "fs"];
times = linspace(0, 10, 100)';
data = zeros(numel(times), numel(names));
trace_time = cell(1, numel(names));
trace_data = cell(1, numel(names));
for index = 1:numel(names)
    signal = out.logsout.getElement(names(index)).Values;
    assert(isvector(signal.Data) && all(isfinite(signal.Data(:))), ...
        "teleop:AnimationSignal", "Animation requires finite scalar signals.");
    assert(signal.Time(1) <= times(1) && signal.Time(end) >= times(end), ...
        "teleop:AnimationTime", "Signal must cover the animation duration.");
    [unique_time, rows] = unique(signal.Time);
    data(:,index) = interp1(unique_time, double(signal.Data(rows)), times, "linear");
    [trace_time{index}, trace_data{index}] = preserve_extrema( ...
        signal.Time(:), double(signal.Data(:)));
end
assert(all(isfinite(data(:))), "teleop:AnimationData", "Invalid animation samples.");
fig = figure("Visible", "off", "Color", "white", ...
    "Position", [100 100 640 440]);
cleanup = onCleanup(@() close(fig));
tiledlayout(3,1, "TileSpacing", "compact", "Padding", "compact");
blue = [.05 .40 .70];
orange = [.85 .35 .10];
motion = nexttile;
hold(motion, "on");
position_limits = padded_limits([trace_data{1}; trace_data{2}]);
xlim(motion, position_limits);
ylim(motion, [0 3]);
yticks(motion, [1 2]);
yticklabels(motion, ["Slave", "Master"]);
grid(motion, "on");
xlabel(motion, "Simulated position [m]");
heading = title(motion, "Four-channel simulation | t = 0.0 s");
master = plot(motion, data(1,1), 2, "o", "MarkerSize", 12, ...
    "MarkerFaceColor", blue, "MarkerEdgeColor", blue);
slave = plot(motion, data(1,2), 1, "s", "MarkerSize", 12, ...
    "MarkerFaceColor", orange, "MarkerEdgeColor", orange);
position = nexttile;
hold(position, "on");
pm = plot(position, times(1), data(1,1), "Color", blue, "LineWidth", 1.5);
ps = plot(position, times(1), data(1,2), "Color", orange, "LineWidth", 1.5);
xlim(position, [0 10]); ylim(position, position_limits);
grid(position, "on"); ylabel(position, "Position [m]");
legend(position, ["Master", "Slave"], "Location", "northwest");
force = nexttile;
hold(force, "on");
fm = plot(force, times(1), data(1,3), "Color", blue, "LineWidth", 1.5);
fs = plot(force, times(1), data(1,4), "Color", orange, "LineWidth", 1.5);
xlim(force, [0 10]); ylim(force, padded_limits([trace_data{3}; trace_data{4}]));
grid(force, "on"); ylabel(force, "Control force [N]"); xlabel(force, "Time [s]");
for frame = 1:numel(times)
    master.XData = data(frame,1);
    slave.XData = data(frame,2);
    heading.String = sprintf("Four-channel simulation | t = %.1f s", times(frame));
    lines = [pm ps fm fs];
    for index = 1:numel(names)
        visible = trace_time{index} <= times(frame);
        set(lines(index), "XData", trace_time{index}(visible), ...
            "YData", trace_data{index}(visible));
    end
    drawnow;
    image = frame2im(getframe(fig));
    [indexed, map] = rgb2ind(image, 64);
    if frame == 1
        imwrite(indexed, map, destination, "gif", "LoopCount", Inf, "DelayTime", .1);
    else
        imwrite(indexed, map, destination, "gif", "WriteMode", "append", "DelayTime", .1);
    end
end
info = dir(destination);
assert(info.bytes <= 5 * 1024^2, "teleop:AnimationSize", ...
    "Animation exceeds the 5 MiB publication limit.");
frames = imfinfo(destination);
assert(numel(frames) == numel(times), "teleop:AnimationFrames", ...
    "Animation must contain every simulation frame.");
end

function limits = padded_limits(values)
low = min(values(:));
high = max(values(:));
margin = max(.08 * (high - low), 1e-3);
limits = [low - margin, high + margin];
end

function [time, values] = preserve_extrema(time, values)
% Keep short force transients that uniform animation sampling would miss.
bucket = floor(640 * time / time(end));
starts = [1; find(diff(bucket) ~= 0) + 1];
stops = [starts(2:end) - 1; numel(time)];
rows = zeros(4 * numel(starts), 1);
for index = 1:numel(starts)
    first = starts(index);
    last = stops(index);
    [~, low] = min(values(first:last));
    [~, high] = max(values(first:last));
    rows(4 * index - 3:4 * index) = [first; first + low - 1; first + high - 1; last];
end
rows = unique(rows);
assert(min(values(rows)) == min(values) && max(values(rows)) == max(values), ...
    "teleop:AnimationExtrema", "Trace reduction must preserve signal extrema.");
time = time(rows);
values = values(rows);
end
