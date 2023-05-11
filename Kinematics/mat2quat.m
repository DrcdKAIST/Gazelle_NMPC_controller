function q = mat2quat(m)
    % q = [w x y z]
    m = m';
    w = 0.5*sqrt(1.0 + m(1,1) + m(2,2) + m(3,3));
    x = sign(m(3,2)-m(2,3))*0.5*sqrt(1+m(1,1)-m(2,2)-m(3,3));
    y = sign(m(1,3)-m(3,1))*0.5*sqrt(1-m(1,1)+m(2,2)-m(3,3));
    z = sign(m(2,1)-m(1,2))*0.5*sqrt(1-m(1,1)-m(2,2)+m(3,3));
    if w*w + x*x + y*y + z*z <0.001 
        w = 1;
    end
    w = real(w);
    x = real(x);
    y = real(y);
    z = real(z);

    q = [w x y z];
end