function add_inports(sys, names, x, y0, dy)
%ADD_INPORTS  Add a column of Inport blocks.
    for i = 1:numel(names)
        y = y0 + dy*(i-1);
        add_block('simulink/Sources/In1', [sys '/' names{i}], ...
            'Position', [x  y  x+30  y+14], 'Port', num2str(i));
    end
end
