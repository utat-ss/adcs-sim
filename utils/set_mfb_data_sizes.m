function set_mfb_data_sizes(blockPath, sizeSpecs)
%SET_MFB_DATA_SIZES  Force MATLAB Function block data sizes and numeric types.
%
%   sizeSpecs is an N-by-2 cell array of {dataName, sizeString}, where
%   sizeString follows Stateflow array-size syntax, e.g. '1', '[3 1]'.

    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    if isempty(chart)
        mdl = bdroot(blockPath);
        set_param(mdl, 'SimulationCommand', 'update');
        chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
    end
    if isempty(chart)
        warning('Could not find chart for %s', blockPath);
        return;
    end

    dataObjs = chart.find('-isa', 'Stateflow.Data');
    for k = 1:size(sizeSpecs, 1)
        dataName = sizeSpecs{k, 1};
        dataSize = sizeSpecs{k, 2};
        idx = find(strcmp({dataObjs.Name}, dataName), 1);
        if ~isempty(idx)
            dataObjs(idx).Props.Array.Size = dataSize;
            dataObjs(idx).Props.Type.Method = 'Built-in';
            dataObjs(idx).Props.Type.Primitive = 'double';
        else
            warning('Could not find data "%s" in %s', dataName, blockPath);
        end
    end
end
