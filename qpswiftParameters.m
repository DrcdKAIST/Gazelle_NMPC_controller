function [P, c, A, b, G, h] = qpswiftParameters(v, t_step, xi_err, xi_ref_horizon, T_step_ref_horizon, dU_x_prev, dU_y_prev)
H = PARA.H;
state_length = PARA.state_length;
input_length = PARA.input_length;
m = PARA.m_all;
g = PARA.g;
w = PARA.w;
dt = PARA.dt_MPC;

xi_err = xi_err(1:2);

% Gain vectors
gain_state_horizon = zeros(H*state_length, 1);
gain_input_horizon = zeros(H*input_length, 1);
for i = 1:H
    gain_state_horizon((i-1)*state_length + 1, 1) = PARA.w_xi_err_x;
    gain_state_horizon((i-1)*state_length + 2, 1) = PARA.w_xi_err_y;
    
    gain_input_horizon((i-1)*input_length + 1, 1) = PARA.w_p_c_x;
    gain_input_horizon((i-1)*input_length + 2, 1) = PARA.w_p_c_y;
    gain_input_horizon((i-1)*input_length + 3, 1) = PARA.w_dU_x;
    gain_input_horizon((i-1)*input_length + 4, 1) = PARA.w_dU_y;
    gain_input_horizon((i-1)*input_length + 5, 1) = PARA.w_db_x;
    gain_input_horizon((i-1)*input_length + 6, 1) = PARA.w_db_y;
    gain_input_horizon((i-1)*input_length + 7, 1) = PARA.w_dT;
    gain_input_horizon((i-1)*input_length + 8, 1) = PARA.w_ddtheta_x;
    gain_input_horizon((i-1)*input_length + 9, 1) = PARA.w_ddtheta_y;
   
end

X = v(1:H*state_length, 1);
X_ref = zeros(H*state_length, 1);
U = v(H*state_length+1:end, 1);

c = J_v_func(gain_state_horizon, gain_input_horizon, X, X_ref, U);
P = J_vv_func(gain_state_horizon, gain_input_horizon, X, X_ref, U);

ceq0 = ceq0_func(xi_err, X, U, m, g, w, dt, PARA.J_x, PARA.J_y);
ceq0_v = ceq0_v_func(xi_err, X, U, m, g, w, dt, PARA.J_x, PARA.J_y);
ceq1 = ceq1_func(xi_err, X, U, m, g, w, dt, t_step, PARA.J_x, PARA.J_y, xi_ref_horizon, T_step_ref_horizon);
ceq1_v = ceq1_v_func(xi_err, X, U, m, g, w, dt, t_step, PARA.J_x, PARA.J_y, xi_ref_horizon, T_step_ref_horizon);

cineq1_max = cineq1_max_func(U, PARA.p_c_x_max, PARA.p_c_y_max);
cineq1_min = cineq1_min_func(U, PARA.p_c_x_min, PARA.p_c_y_min);
cineq2_max = cineq2_max_func(U, PARA.dU_x_max, PARA.dU_y_max);
cineq2_min = cineq2_min_func(U, PARA.dU_x_min, PARA.dU_y_min);
cineq3_max = cineq3_max_func(U, PARA.dT_max);
cineq3_min = cineq3_min_func(U, PARA.dT_min);
cineq4_max = cineq4_max_func(U, PARA.ddtheta_x_max, PARA.ddtheta_y_max);
cineq4_min = cineq4_min_func(U, PARA.ddtheta_x_min, PARA.ddtheta_y_min);
cineq5_max = cineq5_max_func(U, PARA.V_x_max, PARA.V_y_max, dU_x_prev, dU_y_prev, dt);
cineq5_min = cineq5_min_func(U, PARA.V_x_min, PARA.V_y_min, dU_x_prev, dU_y_prev, dt);
cineq6_max = cineq6_max_func(U, PARA.V_x_max, PARA.V_y_max, dU_x_prev, dU_y_prev, PARA.dt);
cineq6_min = cineq6_min_func(U, PARA.V_x_min, PARA.V_y_min, dU_x_prev, dU_y_prev, PARA.dt);
cineq1_max_v = cineq1_max_v_func(U, PARA.p_c_x_max, PARA.p_c_y_max);
cineq1_min_v = cineq1_min_v_func(U, PARA.p_c_x_min, PARA.p_c_y_min);
cineq2_max_v = cineq2_max_v_func(U, PARA.dU_x_max, PARA.dU_y_max);
cineq2_min_v = cineq2_min_v_func(U, PARA.dU_x_min, PARA.dU_y_min);
cineq3_max_v = cineq3_max_v_func(U, PARA.dT_max);
cineq3_min_v = cineq3_min_v_func(U, PARA.dT_min);
cineq4_max_v = cineq4_max_v_func(U, PARA.ddtheta_x_max, PARA.ddtheta_y_max);
cineq4_min_v = cineq4_min_v_func(U, PARA.ddtheta_x_min, PARA.ddtheta_y_min);
cineq5_max_v = cineq5_max_v_func(U, PARA.V_x_max, PARA.V_y_max, dU_x_prev, dU_y_prev, dt);
cineq5_min_v = cineq5_min_v_func(U, PARA.V_x_min, PARA.V_y_min, dU_x_prev, dU_y_prev, dt);
cineq6_max_v = cineq6_max_v_func(U, PARA.V_x_max, PARA.V_y_max, dU_x_prev, dU_y_prev, PARA.dt);
cineq6_min_v = cineq6_min_v_func(U, PARA.V_x_min, PARA.V_y_min, dU_x_prev, dU_y_prev, PARA.dt);

    
    
A = [ceq0_v;
     ceq1_v];
b = (-1).*[ceq0;
           ceq1];
       
G = [cineq1_max_v;
     cineq1_min_v;
     cineq2_max_v;
     cineq2_min_v;
     cineq3_max_v;
     cineq3_min_v;
     cineq4_max_v;
     cineq4_min_v;
     cineq5_max_v;
     cineq5_min_v;
     cineq6_max_v;
     cineq6_min_v];
h = (-1).*[cineq1_max;
           cineq1_min;
           cineq2_max;
           cineq2_min;
           cineq3_max;
           cineq3_min;
           cineq4_max;
           cineq4_min;
           cineq5_max;
           cineq5_min;
           cineq6_max;
           cineq6_min];
end