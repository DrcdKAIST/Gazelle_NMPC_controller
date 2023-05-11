function func = fminconFunc(y, T_step_ref, ddtheta_return)
tau_ref = exp(PARA.w*T_step_ref);
ddtheta_return_x = ddtheta_return(1);
ddtheta_return_y = ddtheta_return(2);

y_ref = [0, 0, 0, 0, tau_ref, ddtheta_return_x, ddtheta_return_y];

W = zeros(length(y_ref), length(y_ref));
W(1, 1) = PARA.w_dU_x_fmincon;
W(2, 2) = PARA.w_dU_y_fmincon;
W(3, 3) = PARA.w_db_x_fmincon;
W(4, 4) = PARA.w_db_y_fmincon;
W(5, 5) = PARA.w_tau_fmincon;
W(6, 6) = PARA.w_ddtheta_x_fmincon;
W(7, 7) = PARA.w_ddtheta_y_fmincon;

func = (y - y_ref)*W*(y - y_ref)';
end