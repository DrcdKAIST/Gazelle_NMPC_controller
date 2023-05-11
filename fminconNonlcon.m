function [c, ceq] = fminconNonlcon(y, xi_err, xi_ref, t_step, T_step_ref, T_step_prev, p_c_star)
    c = [];
    
    tau_ref = exp(PARA.w*T_step_ref);
    tau_prev = exp(PARA.w*T_step_prev);
    xi_err_x = xi_err(1);
    xi_err_y = xi_err(2);
    xi_ref_x = xi_ref(1);
    xi_ref_y = xi_ref(2);
    p_c_star_x = p_c_star(1);
    p_c_star_y = p_c_star(2);
    
    dU_x = y(1);
    dU_y = y(2);
    db_x = y(3);
    db_y = y(4);
    tau = y(5);
    ddtheta_x = y(6);
    ddtheta_y = y(7);
    
    ceq1 = dU_x + db_x + (p_c_star_x - xi_err_x - xi_ref_x)*(exp(-PARA.w*t_step))*tau + (-(1-(exp(-PARA.w*t_step))*tau_prev)*((PARA.J_y/(PARA.m_all*PARA.g)))*ddtheta_y) - p_c_star_x + xi_ref_x*(exp(-PARA.w*t_step))*tau_ref;
    ceq2 = dU_y + db_y + (p_c_star_y - xi_err_y - xi_ref_y)*(exp(-PARA.w*t_step))*tau + (-(1-(exp(-PARA.w*t_step))*tau_prev)*(PARA.J_x/(PARA.m_all*PARA.g)))*ddtheta_x - p_c_star_y + xi_ref_y*(exp(-PARA.w*t_step))*tau_ref;
    
    ceq = [ceq1; ceq2];
end