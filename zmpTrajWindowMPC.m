function p_window = zmpTrajWindowMPC(t_step, step_phase, p_total, T_step_total)
NL = PARA.NL_MPC;

p_window = zeros(3, NL+1);

t_step_temp = t_step; step_phase_temp = step_phase;
T_step_temp = T_step_total(step_phase_temp);
for i = 1:(NL+1)
    if t_step_temp > T_step_temp
        T_step_temp_prev = T_step_temp;
        if step_phase_temp + 1 <= length(p_total)
            step_phase_temp = step_phase_temp + 1;
            T_step_temp = T_step_total(step_phase_temp);
        end
        
        t_step_temp = t_step_temp - T_step_temp_prev;
        if t_step_temp < 1E-06
            t_step_temp = 0;
        end
    end   
    
    p_window(:, i) = p_total(:, step_phase_temp);
    
    t_step_temp = t_step_temp + PARA.dt_MPC;
end
end