function xs = skew(x)
% Convert vector a in R3 to 3x3 skew-symmetric matrix a^x
%   Detailed explanation goes here
arguments (Input)
    x
end

arguments (Output)
    xs
end

xs = [0, -x(3), x(2);
      x(3), 0, -x(1);
      -x(2), x(1), 0];
end