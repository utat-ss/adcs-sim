function s = toSimulinkParams(s)
%TOSIMULINKPARAMS Converts nested struct to Simulink parameters useable in
%simulation model
arguments (Input)
    s
end

arguments (Output)
    s
end

fields = fieldnames(s);
for i = 1:numel(fields)
    val = s.(fields{i});
    if isstruct(val)
        s.(fields{i}) = toSimulinkParams(val);
    elseif isnumeric(val) && isscalar(val);
        s.(fields{i}) = Simulink.Parameter(val);
    end
end
end