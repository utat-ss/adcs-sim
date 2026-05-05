function leaf(parent, name, ins, outs, pos)
%LEAF  Create a stub (blank) subsystem with named inports and outports.
%      Inputs are terminated internally; outputs are driven by Ground blocks.
    fp = [parent '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', fp, 'Position', pos);
    csub(fp);

    sp = 45;                       % vertical pixel spacing
    for i = 1:numel(ins)
        y = 30 + sp*(i-1);
        add_block('simulink/Sources/In1', ...
            [fp '/' ins{i}], ...
            'Position', [50  y  80  y+14], 'Port', num2str(i));
        add_block('simulink/Sinks/Terminator', ...
            [fp '/Term_' num2str(i)], ...
            'Position', [160 y  180 y+14]);
        add_line(fp, [ins{i} '/1'], ['Term_' num2str(i) '/1']);
    end
    for i = 1:numel(outs)
        y = 30 + sp*(i-1);
        add_block('simulink/Sources/Ground', ...
            [fp '/Gnd_' num2str(i)], ...
            'Position', [260 y  280 y+14]);
        add_block('simulink/Sinks/Out1', ...
            [fp '/' outs{i}], ...
            'Position', [360 y  390 y+14], 'Port', num2str(i));
        add_line(fp, ['Gnd_' num2str(i) '/1'], [outs{i} '/1']);
    end
end
