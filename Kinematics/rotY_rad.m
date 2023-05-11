function rotM = rotY_rad(q)
    rotM = [ cos(q) 0 sin(q);
             0      1 0;
            -sin(q) 0 cos(q)];
end