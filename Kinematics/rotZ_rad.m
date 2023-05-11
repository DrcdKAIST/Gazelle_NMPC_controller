function rotM = rotZ_rad(q)
    rotM = [ cos(q) -sin(q) 0;
             sin(q)  cos(q) 0;
             0       0      1];
end