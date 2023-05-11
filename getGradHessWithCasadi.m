addpath('casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

PARA = PARA;

H = PARA.H;
state_length = PARA.state_length;
input_length = PARA.input_length;

gain_state_horizon = SX.sym('gain_state_horizon', (H*state_length));
gain_input_horizon = SX.sym('gain_input_horizon', H*input_length);

dt = SX.sym('dt'); dt_real = SX.sym('dt_real');
g = SX.sym('g');
w = SX.sym('w');
m = SX.sym('m');
t_step = SX.sym('t_step');
J_x = SX.sym('J_x');
J_y = SX.sym('J_y');
V_x_max = SX.sym('V_x_max'); V_x_min = SX.sym('V_x_min');
V_y_max = SX.sym('V_y_max'); V_y_min = SX.sym('V_y_min');
dU_x_prev = SX.sym('dU_x_prev');
dU_y_prev = SX.sym('dU_y_prev');

p_c_x_max = SX.sym('p_c_x_max');
p_c_y_max = SX.sym('p_c_y_max');
p_c_x_min = SX.sym('p_c_x_min');
p_c_y_min = SX.sym('p_c_y_min');
dU_x_max = SX.sym('dU_x_max');
dU_y_max = SX.sym('dU_y_max');
dU_x_min = SX.sym('dU_x_min');
dU_y_min = SX.sym('dU_y_min');
dT_max = SX.sym('dT_max');
dT_min = SX.sym('dT_min');
ddtheta_x_max = SX.sym('ddtheta_x_max');
ddtheta_y_max = SX.sym('ddtheta_y_max');
ddtheta_x_min = SX.sym('ddtheta_x_min');
ddtheta_y_min = SX.sym('ddtheta_y_min');

xi_ref_horizon = SX.sym('xi_ref_horizon', 3, H);
T_step_ref_horizon = SX.sym('T_step_ref_horizon', H);

X = SX.sym('X', H*state_length);
X_ref = SX.sym('X_ref', H*state_length);
U = SX.sym('U', H*input_length);
v = [X; U];
x = SX.sym('x', state_length);


% Cost function
J = (X-X_ref)'*diag(gain_state_horizon)*(X-X_ref) + U'*diag(gain_input_horizon)*U;
J_v = jacobian(J, v);
J_vv = hessian(J, v);

% Equality constraints
ceq0 = [];
xi_err = x;
for i = 1:H
    p_c_ZMP = [U((i-1)*input_length + 1); U((i-1)*input_length + 2)];
    
    ddtheta = [U((i-1)*input_length + 8); U((i-1)*input_length + 9)];
    p_c_CMP = [(J_y*ddtheta(2))/(m*g); (J_x*ddtheta(1))/(m*g)];
    
    p_c = p_c_ZMP + p_c_CMP;
    
    xi_err_next = (1 + w*dt).*xi_err - (w*dt).*p_c; 
    ceq0_sub = X((i-1)*state_length + 1 : (i-1)*state_length + 2) - xi_err_next;
    
    ceq0 = [ceq0; ceq0_sub];
    
    xi_err = xi_err_next;    
end

ceq1 = [];
xi_err_x = x(1);
xi_err_y = x(2);
for i = 1:H
    p_c_x = U((i-1)*input_length + 1);
    p_c_y = U((i-1)*input_length + 2);
    dU_x = U((i-1)*input_length + 3);
    dU_y = U((i-1)*input_length + 4);
    db_x = U((i-1)*input_length + 5);
    db_y = U((i-1)*input_length + 6);
    dT = U((i-1)*input_length + 7);
    ddtheta_x = U((i-1)*input_length + 8);
    ddtheta_y = U((i-1)*input_length + 9);
    
    xi_ref_x = xi_ref_horizon(1, i);
    xi_ref_y = xi_ref_horizon(2, i);
    T_step_ref = T_step_ref_horizon(i);
    
    T_step = T_step_ref + dT;
    
    ceq1_x_sub = dU_x + db_x + xi_ref_x*exp(-w*t_step)*(exp(w*T_step_ref)-exp(w*T_step)) - (1 - exp(w*(T_step - t_step)))*((J_y*ddtheta_y)/(m*g)) - ((xi_err_x - p_c_x)*exp(w*(T_step - t_step)) + p_c_x);
    ceq1_y_sub = dU_y + db_y + xi_ref_y*exp(-w*t_step)*(exp(w*T_step_ref)-exp(w*T_step)) - (1 - exp(w*(T_step - t_step)))*((J_x*ddtheta_x)/(m*g)) - ((xi_err_y - p_c_y)*exp(w*(T_step - t_step)) + p_c_y);
    
    ceq1 = [ceq1; ceq1_x_sub; ceq1_y_sub];
    
    xi_err_x = X((i-1)*state_length + 1);
    xi_err_y = X((i-1)*state_length + 2);
end

ceq0_v = jacobian(ceq0, v);
ceq1_v = jacobian(ceq1, v);

% Inequality constraints
cineq1_max = [];
for i = 1:H
    p_c_x = U((i-1)*input_length + 1);
    p_c_y = U((i-1)*input_length + 2);
    
    cineq1_max_x_sub = p_c_x - p_c_x_max;
    cineq1_max_y_sub = p_c_y - p_c_y_max;
    
    cineq1_max = [cineq1_max; cineq1_max_x_sub; cineq1_max_y_sub];
end

cineq1_min = [];
for i = 1:H
    p_c_x = U((i-1)*input_length + 1);
    p_c_y = U((i-1)*input_length + 2);
    
    cineq1_min_x_sub = -p_c_x + p_c_x_min;
    cineq1_min_y_sub = -p_c_y + p_c_y_min;
    
    cineq1_min = [cineq1_min; cineq1_min_x_sub; cineq1_min_y_sub];
end

cineq2_max = [];
for i = 1:H
    dU_x = U((i-1)*input_length + 3);
    dU_y = U((i-1)*input_length + 4);
    
    cineq2_max_x_sub = dU_x - dU_x_max;
    cineq2_max_y_sub = dU_y - dU_y_max;
    
    cineq2_max = [cineq2_max; cineq2_max_x_sub; cineq2_max_y_sub];
end

cineq2_min = [];
for i = 1:H
    dU_x = U((i-1)*input_length + 3);
    dU_y = U((i-1)*input_length + 4);
    
    cineq2_min_x_sub = -dU_x + dU_x_min;
    cineq2_min_y_sub = -dU_y + dU_y_min;
    
    cineq2_min = [cineq2_min; cineq2_min_x_sub; cineq2_min_y_sub];
end

cineq3_max = [];
for i = 1:H
    dT = U((i-1)*input_length + 7);
    
    cineq3_max_sub = dT - dT_max;
    
    cineq3_max = [cineq3_max; cineq3_max_sub];
end

cineq3_min = [];
for i = 1:H
    dT = U((i-1)*input_length + 7);
    
    cineq3_min_x_sub = -dT + dT_min;
    
    cineq3_min = [cineq3_min; cineq3_min_x_sub];
end

cineq4_max = [];
for i = 1:H
    ddtheta_x = U((i-1)*input_length + 8);
    ddtheta_y = U((i-1)*input_length + 9);
    
    cineq4_max_x_sub = ddtheta_x - ddtheta_x_max;
    cineq4_max_y_sub = ddtheta_y - ddtheta_y_max;
    
    cineq4_max = [cineq4_max; cineq4_max_x_sub; cineq4_max_y_sub];
end

cineq4_min = [];
for i = 1:H
    ddtheta_x = U((i-1)*input_length + 8);
    ddtheta_y = U((i-1)*input_length + 9);
    
    cineq4_min_x_sub = -ddtheta_x + ddtheta_x_min;
    cineq4_min_y_sub = -ddtheta_y + ddtheta_y_min;
    
    cineq4_min = [cineq4_min; cineq4_min_x_sub; cineq4_min_y_sub];
end

cineq5_max = [];
for i = 1:H
    dU_x = U((i-1)*input_length + 3);
    dU_y = U((i-1)*input_length + 4);
    
    cineq5_max_x_sub = dU_x - dU_x_prev - i*V_x_max*dt;
    cineq5_max_y_sub = dU_y - dU_y_prev - i*V_y_max*dt;
    
    cineq5_max = [cineq5_max; cineq5_max_x_sub; cineq5_max_y_sub];
end

cineq5_min = [];
for i = 1:H
    dU_x = U((i-1)*input_length + 3);
    dU_y = U((i-1)*input_length + 4);
    
    cineq5_min_x_sub = -(dU_x - dU_x_prev) + i*V_x_min*dt;
    cineq5_min_y_sub = -(dU_y - dU_y_prev) + i*V_y_min*dt;
        
    cineq5_min = [cineq5_min; cineq5_min_x_sub; cineq5_min_y_sub];
end

% cineq6_max = [];
dU_x = U(3);
dU_y = U(4);
cineq6_max_x_sub = dU_x - dU_x_prev - V_x_max*dt_real;
cineq6_max_y_sub = dU_y - dU_y_prev - V_y_max*dt_real;
cineq6_max = [cineq6_max_x_sub; cineq6_max_y_sub];

% cineq6_min = [];
dU_x = U(3);
dU_y = U(4);
cineq6_min_x_sub = -(dU_x - dU_x_prev) + V_x_min*dt_real;
cineq6_min_y_sub = -(dU_y - dU_y_prev) + V_y_min*dt_real;
cineq6_min = [cineq6_min_x_sub; cineq6_min_y_sub];

cineq1_max_v = jacobian(cineq1_max, v);
cineq1_min_v = jacobian(cineq1_min, v);
cineq2_max_v = jacobian(cineq2_max, v);
cineq2_min_v = jacobian(cineq2_min, v);
cineq3_max_v = jacobian(cineq3_max, v);
cineq3_min_v = jacobian(cineq3_min, v);
cineq4_max_v = jacobian(cineq4_max, v);
cineq4_min_v = jacobian(cineq4_min, v);
cineq5_max_v = jacobian(cineq5_max, v);
cineq5_min_v = jacobian(cineq5_min, v);
cineq6_max_v = jacobian(cineq6_max, v);
cineq6_min_v = jacobian(cineq6_min, v);

%--- Function generation
opts=struct('main',true,'mex',true,'with_header',true);

% Cost function
J_v_func = Function('J_v_func', {gain_state_horizon, gain_input_horizon, X, X_ref, U}, {J_v});
J_vv_func = Function('J_vv_func', {gain_state_horizon, gain_input_horizon, X, X_ref, U}, {J_vv});

J_v_func.generate('J_v_func.c', opts);
mex  J_v_func.c
J_vv_func.generate('J_vv_func.c', opts);
mex  J_vv_func.c

% Equality constraints
ceq0_func = Function('ceq0_func', {x, X, U, m, g, w, dt, J_x, J_y}, {ceq0});
ceq1_func = Function('ceq1_func', {x, X, U, m, g, w, dt, t_step, J_x, J_y, xi_ref_horizon, T_step_ref_horizon}, {ceq1});
ceq0_v_func = Function('ceq0_v_func', {x, X, U, m, g, w, dt, J_x, J_y}, {ceq0_v});
ceq1_v_func = Function('ceq1_v_func', {x, X, U, m, g, w, dt, t_step, J_x, J_y, xi_ref_horizon, T_step_ref_horizon}, {ceq1_v});

ceq0_func.generate('ceq0_func.c', opts);
mex  ceq0_func.c
ceq0_v_func.generate('ceq0_v_func.c', opts);
mex  ceq0_v_func.c
ceq1_func.generate('ceq1_func.c', opts);
mex  ceq1_func.c
ceq1_v_func.generate('ceq1_v_func.c', opts);
mex  ceq1_v_func.c

% Inequality constraints
cineq1_max_func = Function('cineq1_max_func', {U, p_c_x_max, p_c_y_max}, {cineq1_max});
cineq1_min_func = Function('cineq1_min_func', {U, p_c_x_min, p_c_y_min}, {cineq1_min});
cineq2_max_func = Function('cineq2_max_func', {U, dU_x_max, dU_y_max}, {cineq2_max});
cineq2_min_func = Function('cineq2_min_func', {U, dU_x_min, dU_y_min}, {cineq2_min});
cineq3_max_func = Function('cineq3_max_func', {U, dT_max}, {cineq3_max});
cineq3_min_func = Function('cineq3_min_func', {U, dT_min}, {cineq3_min});
cineq4_max_func = Function('cineq4_max_func', {U, ddtheta_x_max, ddtheta_y_max}, {cineq4_max});
cineq4_min_func = Function('cineq4_min_func', {U, ddtheta_x_min, ddtheta_y_min}, {cineq4_min});
cineq5_max_func = Function('cineq5_max_func', {U, V_x_max, V_y_max, dU_x_prev, dU_y_prev, dt}, {cineq5_max});
cineq5_min_func = Function('cineq5_min_func', {U, V_x_min, V_y_min, dU_x_prev, dU_y_prev, dt}, {cineq5_min});
cineq6_max_func = Function('cineq6_max_func', {U, V_x_max, V_y_max, dU_x_prev, dU_y_prev, dt_real}, {cineq6_max});
cineq6_min_func = Function('cineq6_min_func', {U, V_x_min, V_y_min, dU_x_prev, dU_y_prev, dt_real}, {cineq6_min});
cineq1_max_v_func = Function('cineq1_max_v_func', {U, p_c_x_max, p_c_y_max}, {cineq1_max_v});
cineq1_min_v_func = Function('cineq1_min_v_func', {U, p_c_x_min, p_c_y_min}, {cineq1_min_v});
cineq2_max_v_func = Function('cineq2_max_v_func', {U, dU_x_max, dU_y_max}, {cineq2_max_v});
cineq2_min_v_func = Function('cineq2_min_v_func', {U, dU_x_min, dU_y_min}, {cineq2_min_v});
cineq3_max_v_func = Function('cineq3_max_v_func', {U, dT_max}, {cineq3_max_v});
cineq3_min_v_func = Function('cineq3_min_v_func', {U, dT_min}, {cineq3_min_v});
cineq4_max_v_func = Function('cineq4_max_v_func', {U, ddtheta_x_max, ddtheta_y_max}, {cineq4_max_v});
cineq4_min_v_func = Function('cineq4_min_v_func', {U, ddtheta_x_min, ddtheta_y_min}, {cineq4_min_v});
cineq5_max_v_func = Function('cineq5_max_v_func', {U, V_x_max, V_y_max, dU_x_prev, dU_y_prev, dt}, {cineq5_max_v});
cineq5_min_v_func = Function('cineq5_min_v_func', {U, V_x_min, V_y_min, dU_x_prev, dU_y_prev, dt}, {cineq5_min_v});
cineq6_max_v_func = Function('cineq6_max_v_func', {U, V_x_max, V_y_max, dU_x_prev, dU_y_prev, dt_real}, {cineq6_max_v});
cineq6_min_v_func = Function('cineq6_min_v_func', {U, V_x_min, V_y_min, dU_x_prev, dU_y_prev, dt_real}, {cineq6_min_v});

cineq1_max_func.generate('cineq1_max_func.c', opts);
mex  cineq1_max_func.c
cineq1_min_func.generate('cineq1_min_func.c', opts);
mex  cineq1_min_func.c
cineq2_max_func.generate('cineq2_max_func.c', opts);
mex  cineq2_max_func.c
cineq2_min_func.generate('cineq2_min_func.c', opts);
mex  cineq2_min_func.c
cineq3_max_func.generate('cineq3_max_func.c', opts);
mex  cineq3_max_func.c
cineq3_min_func.generate('cineq3_min_func.c', opts);
mex  cineq3_min_func.c
cineq4_max_func.generate('cineq4_max_func.c', opts);
mex  cineq4_max_func.c
cineq4_min_func.generate('cineq4_min_func.c', opts);
mex  cineq4_min_func.c
cineq5_max_func.generate('cineq5_max_func.c', opts);
mex  cineq5_max_func.c
cineq5_min_func.generate('cineq5_min_func.c', opts);
mex  cineq5_min_func.c
cineq6_max_func.generate('cineq6_max_func.c', opts);
mex  cineq6_max_func.c
cineq6_min_func.generate('cineq6_min_func.c', opts);
mex  cineq6_min_func.c
cineq1_max_v_func.generate('cineq1_max_v_func.c', opts);
mex  cineq1_max_v_func.c
cineq1_min_v_func.generate('cineq1_min_v_func.c', opts);
mex  cineq1_min_v_func.c
cineq2_max_v_func.generate('cineq2_max_v_func.c', opts);
mex  cineq2_max_v_func.c
cineq2_min_v_func.generate('cineq2_min_v_func.c', opts);
mex  cineq2_min_v_func.c
cineq3_max_v_func.generate('cineq3_max_v_func.c', opts);
mex  cineq3_max_v_func.c
cineq3_min_v_func.generate('cineq3_min_v_func.c', opts);
mex  cineq3_min_v_func.c
cineq4_max_v_func.generate('cineq4_max_v_func.c', opts);
mex  cineq4_max_v_func.c
cineq4_min_v_func.generate('cineq4_min_v_func.c', opts);
mex  cineq4_min_v_func.c
cineq5_max_v_func.generate('cineq5_max_v_func.c', opts);
mex  cineq5_max_v_func.c
cineq5_min_v_func.generate('cineq5_min_v_func.c', opts);
mex  cineq5_min_v_func.c
cineq6_max_v_func.generate('cineq6_max_v_func.c', opts);
mex  cineq6_max_v_func.c
cineq6_min_v_func.generate('cineq6_min_v_func.c', opts);
mex  cineq6_min_v_func.c
%---