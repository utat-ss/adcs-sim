function vx = skew3(v)
%SKEW3 Return the skew-symmetric form of a vector in R3
%   Skew-symmetric form is equivalent to using a cross operator
arguments (Input)
    v       % Vector in R3 (3x1)
end

arguments (Output)
    vx      % Skew-symmetric matrix of vector (3x3)
end

vx = [0,        -v(3),  v(2);
      v(3),     0,      -v(1);
      -v(2),    v(1),   0];
end