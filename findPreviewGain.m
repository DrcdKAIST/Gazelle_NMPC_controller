function [Gi,Gx,Gp] = findPreviewGain(preview_time, dt, zc)
    hz = 1/dt;
    NL = round(preview_time*hz);
    g = 9.81;
    w = sqrt(g/zc);
    
    A = [1, dt, (dt*dt)/2; 0, 1, dt; 0, 0, 1];
    B = [(dt*dt*dt)/6; (dt*dt)/2; dt];
    C = [1, 0, -(1/(w^2))];
    
    Qe = 10;
    Qx = [0 0 0; 0 0 0 ; 0 0 0.0];
    R = 1*10^-2;

    B_t = [C*B;B];
    I_t = [1;0 ;0 ;0];
    F_t = [C*A;A];
    Q_t = [Qe 0 0 0; [0 0 0]' Qx];
    A_t = [I_t F_t];

    [K_t,L,G] = dare(A_t,B_t,Q_t,R);

    Gi = inv(R+B_t'*K_t*B_t)*B_t'*K_t*I_t;
    Gx = inv(R+B_t'*K_t*B_t)*B_t'*K_t*F_t;

    Ac_t = A_t - B_t*inv(R+B_t'*K_t*B_t)*B_t'*K_t*A_t;

    X_t(:,1) = -Ac_t'*K_t*I_t;

    for l = 2:NL
        X_t(:,l) = Ac_t'*X_t(:,l-1);
    end

    Gp(:,1) = -Gi;
    for l=2:NL
        Gp(:,l) = inv(R+B_t'*K_t*B_t)*B_t'*X_t(:,l-1);
    end
end