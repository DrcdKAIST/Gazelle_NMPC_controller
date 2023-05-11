function [COM_next, dCOM_next, ddCOM_next, p_err_sum_x_next, p_err_sum_y_next] = ...
    previewControlMPC(t_step, step_phase, p_total, T_step_total, Gi, Gx, Gp, A, B, C, COM, dCOM, ddCOM, p_err_sum_x, p_err_sum_y)

    COM_x_dx_ddx = [COM(1); dCOM(1); ddCOM(1)];
    COM_y_dy_ddy = [COM(2); dCOM(2); ddCOM(2)];
    
    p_window = zmpTrajWindowMPC(t_step, step_phase, p_total, T_step_total);
    
    temp_x1 = 0;    temp_x2 = 0;
    temp_y1 = 0;    temp_y2 = 0;
    for k1 = 1:3
        temp_x1 = temp_x1 + Gx(k1)*COM_x_dx_ddx(k1);
        temp_y1 = temp_y1 + Gx(k1)*COM_y_dy_ddy(k1);
    end
    for k2 = 1:PARA.NL_MPC
        temp_x2 = temp_x2 + p_window(1, k2+1)*Gp(k2);
        temp_y2 = temp_y2 + p_window(2, k2+1)*Gp(k2);
    end
    
    u_x = -Gi*p_err_sum_x - temp_x1 - temp_x2;
    u_y = -Gi*p_err_sum_y - temp_y1 - temp_y2;
    COM_x_dx_ddx_next = A*COM_x_dx_ddx + B*u_x;
    COM_y_dy_ddy_next = A*COM_y_dy_ddy + B*u_y;
    COM_next = [COM_x_dx_ddx_next(1); COM_y_dy_ddy_next(1); PARA.zc];
    dCOM_next = [COM_x_dx_ddx_next(2); COM_y_dy_ddy_next(2); 0];
    ddCOM_next = [COM_x_dx_ddx_next(3); COM_y_dy_ddy_next(3); 0];
    
    p_x = C*COM_x_dx_ddx;
    p_y = C*COM_y_dy_ddy;
    p_err_sum_x_next = p_err_sum_x + (p_x - p_window(1, 1));
    p_err_sum_y_next = p_err_sum_y + (p_y - p_window(2, 1));
end