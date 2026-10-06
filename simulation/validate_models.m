function validate_models()
%VALIDATE_MODELS Compile and run each model, rejecting non-finite output.
names = ["FourCh_TDPA_TD", "PP_TDPA_TD", "PFmsr_TDPA_TD"];
for name = names
    out = run_model(name, 2);
    assert(strlength(string(out.ErrorMessage)) == 0, ...
        "teleop:SimulationError", "Simulation error: %s", out.ErrorMessage);
    assert(out.tout(end) >= 2 - 1e-9, ...
        "teleop:IncompleteSimulation", "Model stopped early: %s", name);
    signals = out.logsout;
    checked = signals.numElements;
    assert(checked > 0, "teleop:NoSignals", "No signals logged: %s", name);
    for i = 1:checked
        signal = signals.getElement(i);
        assert(isa(signal.Values, "timeseries") && ~isempty(signal.Values.Data), ...
            "teleop:InvalidSignal", "%s: missing timeseries %s", name, signal.Name);
        assert(all(isfinite(signal.Values.Data(:))), ...
            "teleop:NonFiniteSignal", "%s: non-finite %s", name, signal.Name);
    end
    fprintf("%s: ran to %.3f s; %d finite timeseries checked.\n", ...
        name, out.tout(end), checked);
end
end
