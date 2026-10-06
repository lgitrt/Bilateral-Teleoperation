function out = run_model(model_name, stop_time)
%RUN_MODEL Run a published teleoperation model without machine-specific paths.
arguments
    model_name (1,1) string {mustBeMember(model_name, ...
        ["FourCh_TDPA_TD", "PP_TDPA_TD", "PFmsr_TDPA_TD"])} = "FourCh_TDPA_TD"
    stop_time (1,1) double {mustBePositive, mustBeFinite} = 10
end
here = fileparts(mfilename("fullpath"));
model_file = fullfile(here, "models", model_name + ".slx");
assert(isfile(model_file), "teleop:MissingModel", "Model not found: %s", model_file);
assert(~bdIsLoaded(model_name), "teleop:ModelAlreadyOpen", ...
    "Close %s before running this wrapper; existing unsaved edits must not be discarded.", model_name);
load_system(model_file);
cleanup = onCleanup(@() close_system(model_name, 0));
lines = find_system(model_name, "FindAll", "on", "SearchDepth", 1, "Type", "line");
for i = 1:numel(lines)
    port = get_param(lines(i), "SrcPortHandle");
    if port == -1
        continue
    end
    if strlength(string(get_param(lines(i), "Name"))) == 0
        block = get_param(port, "Parent");
        label = matlab.lang.makeValidName( ...
            string(get_param(block, "Name")) + "_" + get_param(port, "PortNumber"));
        set_param(port, "DataLoggingNameMode", "Custom", "DataLoggingName", label);
    end
    set_param(port, "DataLogging", "on");
end
input = Simulink.SimulationInput(model_name);
input = input.setModelParameter("StopTime", num2str(stop_time), ...
    "ReturnWorkspaceOutputs", "on", "SignalLogging", "on", ...
    "SignalLoggingName", "logsout");
out = sim(input);
end
