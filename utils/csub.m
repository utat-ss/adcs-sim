function csub(path)
%CSUB  Clear the default In1→Out1 inside a freshly-added Subsystem block.
    try delete_line(path, 'In1/1', 'Out1/1'); catch, end
    try delete_block([path '/In1']);           catch, end
    try delete_block([path '/Out1']);          catch, end
end
