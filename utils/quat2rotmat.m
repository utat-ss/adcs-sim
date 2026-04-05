function C = quat2rotmat(q)
%QUAT2ROTMAT Convert a unit quaternion to a rotation matrix
%   Uses scalar-last JPL convention [qx; qy; qz; qw]
arguments (Input)
    q       % JPL unit quaternion describing Frame A to Frame B rotation
end

arguments (Output)
    C       % Direction Cosine Matrix describing Frame A to Frame B rotation
end

% Ensure q is column vector and normalized
q = q(:) / norm(q);

qx = q(1);
qy = q(2);
qz = q(3);
qw = q(4);

% Pre-calculate squares for efficiency
qxx = qx*qx; qyy = qy*qy; qzz = qz*qz; qww = qw*qw;
qxy = qx*qy; qxz = qx*qz; qxw = qx*qw;
qyz = qy*qz; qyw = qy*qw; qzw = qz*qw;

% Construct DCM from Frame A to frame B
C = [qww + qxx - qyy - qzz, 2*(qxy + qzw),          2*(qxz - qyw);
     2*(qxy - qzw),         qww - qxx + qyy - qzz,  2*(qyz + qzw);
     2*(qxz + qyw),         2*(qyz - qxw),          qww - qxx - qyy + qzz];
end