function m = quat2mat(q)
    m = zeros(3, 3);
    % q = [w x y z]
    xx = 2*q(2)*q(2); yy = 2*q(3)*q(3); zz = 2*q(4)*q(4);
    wx = 2*q(1)*q(2); wy = 2*q(1)*q(3); wz = 2*q(1)*q(4);
	xy = 2*q(2)*q(3); yz = 2*q(3)*q(4); zx = 2*q(4)*q(2);

    m(1,1) = 1.0-yy-zz; m(1,2) =      xy-wz; m(1,3) =      zx+wy;
	m(2,1) =      xy+wz; m(2,2) = 1.0-xx-zz; m(2,3) =      yz-wx;
	m(3,1) =      zx-wy; m(3,2) =      yz+wx; m(3,3) = 1.0-xx-yy;

end