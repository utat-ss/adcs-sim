function qp = quatmult(q1, q2)
%QUATMULT Quaternion multiplication
%   q1, q2, and qp must all be scalar-last JPL convention
arguments (Input)
    q1          % First unit quaternion
    q2          % Second unit quaternion
end

arguments (Output)
    qp          % Quaternion product
end

q1v = [q1(1); q1(2); q1(3)]; q1w = q1(4);
q2v = [q2(1); q2(2); q2(3)]; q2w = q2(4);

qp = [q1w*q2v + q2w*q1v 1 cross(q1v, q2v);
      q1w*q2w - dot(q1v, q2v)];
end