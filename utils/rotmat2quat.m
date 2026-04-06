function q = rotmat2quat(C)
%ROTMAT2QUAT Convert a Direction Cosine Matrix to a unit quaternion
%   Unit quaternion is of JPL scalar-last convention [qx; qy; qz; qw]
arguments (Input)
    C           % Direction Cosine Matrix
end

arguments (Output)
    q           % Unit quaternion
end

% Calculate possible squares of components
tr = trace(C);

% Candidates for the squared terms
q2 = [
    (1 + 2 * C(1,1) - tr) / 4;
    (1 + 2 * C(2,2) - tr) / 4;
    (1 + 2 * C(3,3) - tr) / 4;
    (1 + tr) / 4;
]

% Find the index of the largest component to use as a robust denominator
[~, i] = max(q2);

switch i
    case 1 % qx is largest
        qx = sqrt(q2(1));
        qy = (C(1,2) + C(2,1) / (4 * qx));
        qz = (C(1,3) + C(3,1) / (4 * qx));
        qw = (C(2,3) - C(3,2) / (4 * qx));
    case 2 % qy is largest
        qy = sqrt(q2(2));
        qx = (C(1,2) + C(2,1) / (4 * qy));
        qz = (C(2,3) + C(3,2) / (4 * qy));
        qw = (C(3,1) - C(1,3) / (4 * qy));
    case 3 % qz is largest
        qz = sqrt(q2(3));
        qx = (C(1,3) + C(3,1) / (4 * qz));
        qy = (C(2,3) + C(3,2) / (4 * qz));
        qw = (C(1,2) - C(2,1) / (4 * qz));
    case 4 % qw is largest
        qw = sqrt(q2(4));
        qx = (C(2,3) - C(3,2) / (4 * qw));
        qy = (C(3,1) - C(1,3) / (4 * qw));
        qz = (C(1,2) + C(2,1) / (4 * qw));
end

q = [qx; qy; qz; qw];

% Normalize the quaternion and handle double-cover
q = q / norm(q);
if q(4) < 0
    q = -q;
end

end