function [p_c, dU, db, dT, ddtheta] = nextState(v0, t_step, xi_err, xi_ref_horizon, T_step_ref_horizon, dU_x_prev, dU_y_prev)

iter = 0;
v = v0;
dv = 1E03;
update_rate = 1.0;

while norm(dv) >= 1e00    
    [P, c, A, b, G, h] = qpswiftParameters(v, t_step, xi_err, xi_ref_horizon, T_step_ref_horizon, dU_x_prev, dU_y_prev);
    [dv, ~, ~] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);
    v = v + update_rate*dv;
    
    if iter >= 1e02
        fprintf("Iteration Exceed!\n");
        norm(dv)
        break;
    end
    if sum(isnan(dv)) > 0
        disp(v);
        fprintf("NAN!\n");
        break;
    end
    
    iter = iter + 1;
end

p_c = v(PARA.H*PARA.state_length + 1 : PARA.H*PARA.state_length + 2);
dU = v(PARA.H*PARA.state_length + 3 : PARA.H*PARA.state_length + 4);
db = v(PARA.H*PARA.state_length + 5 : PARA.H*PARA.state_length + 6);
dT = v(PARA.H*PARA.state_length + 7);
ddtheta = v(PARA.H*PARA.state_length + 8 : PARA.H*PARA.state_length + 9);
end