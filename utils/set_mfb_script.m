function set_mfb_script(blockPath, scriptStr)
%SET_MFB_SCRIPT  Set the embedded MATLAB code of a MATLAB Function block.
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    if isempty(chart)
        % Force diagram update so Stateflow chart object is created
        mdl = bdroot(blockPath);
        set_param(mdl, 'SimulationCommand', 'update');
        chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    end
    if ~isempty(chart)
        chart.Script = scriptStr;
    else
        warning('Could not set script for %s', blockPath);
    end
end
