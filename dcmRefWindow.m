function [xi_ref_horizon] = dcmRefWindow(t_step, step_phase, COM_ref, dCOM_ref, ddCOM_ref, p_err_sum_x_ref, p_err_sum_y_ref, T_step_ref, p_ref_total, T_step_ref_total, Gi_MPC, Gx_MPC, Gp_MPC)
    xi_ref_horizon = zeros(3, PARA.H);    

    t_step_temp = t_step; T_step_ref_temp = T_step_ref;
    step_phase_temp = step_phase;
    COM_ref_temp = COM_ref; dCOM_ref_temp = dCOM_ref; ddCOM_ref_temp = ddCOM_ref;
    p_err_sum_x_ref_temp = p_err_sum_x_ref; p_err_sum_y_ref_temp = p_err_sum_y_ref;
    for j = 1:PARA.H
        flag_STEP_CHANGE_TEMP = checkStepEnd(t_step_temp, T_step_ref_temp);
        if flag_STEP_CHANGE_TEMP == 1
            T_step_ref_temp_prev = T_step_ref_temp;
            % Update step phase
            if step_phase_temp < length(p_ref_total)
                step_phase_temp = step_phase_temp + 1;
            end
            
            % Update step time
            T_step_ref_temp = T_step_ref_total(:, step_phase_temp);
            
            % Reset t_step & flag
            t_step_temp = t_step_temp - T_step_ref_temp_prev;
            if t_step_temp < 1E-06
                t_step_temp = 0;
            end
        end
                
        [COM_ref_temp_next, dCOM_ref_temp_next, ddCOM_ref_temp_next, p_err_sum_x_ref_temp_next, p_err_sum_y_ref_temp_next] = previewControlMPC(t_step_temp, step_phase_temp, p_ref_total, T_step_ref_total, Gi_MPC, Gx_MPC, Gp_MPC, PARA.A_preview_MPC, PARA.B_preview_MPC, PARA.C_preview_MPC, COM_ref_temp, dCOM_ref_temp, ddCOM_ref_temp, p_err_sum_x_ref_temp, p_err_sum_y_ref_temp);  
        
        xi_ref_horizon(:, j) = COM_ref_temp + (dCOM_ref_temp./PARA.w);
        
        t_step_temp = t_step_temp + PARA.dt_MPC;
        
        COM_ref_temp = COM_ref_temp_next; dCOM_ref_temp = dCOM_ref_temp_next; ddCOM_ref_temp = ddCOM_ref_temp_next;
        p_err_sum_x_ref_temp = p_err_sum_x_ref_temp_next; p_err_sum_y_ref_temp = p_err_sum_y_ref_temp_next;
    end
end