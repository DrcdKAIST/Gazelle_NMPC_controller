function p_horizon = zmpHorizon(t_step, step_phase, p_total, T_step_total)
H = PARA.H;

p_horizon = zeros(3, H);

t_step_temp = t_step; step_phase_temp = step_phase;
T_step_temp = T_step_total(step_phase_temp);
for i = 1:H
    if t_step_temp > T_step_temp
        if step_phase_temp + 1 <= length(p_total)
            step_phase_temp = step_phase_temp + 1;
            T_step_temp = T_step_total(step_phase_temp);
        end
        
        t_step_temp = t_step_temp - T_step_temp;
        if t_step_temp < 1E-06
            t_step_temp = 0;
        end
    end   
    
    p_horizon(:, i) = p_total(:, step_phase_temp);
    
    t_step_temp = t_step_temp + PARA.dt_MPC;
end
end