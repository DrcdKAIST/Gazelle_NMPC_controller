function t_step_horizon = tStepHorizon(t_step, step_phase, T_step_total)
H = PARA.H;

t_step_horizon = zeros(1, H);

t_step_temp = t_step; step_phase_temp = step_phase;
T_step_temp = T_step_total(step_phase_temp);
for i = 1:H
    if t_step_temp > T_step_temp
        T_step_temp_prev = T_step_temp;
        if step_phase_temp + 1 <= length(T_step_temp)
            step_phase_temp = step_phase_temp + 1;
            T_step_temp = T_step_total(step_phase_temp);
        end
        
        t_step_temp = t_step_temp - T_step_temp_prev;
        if t_step_temp < 1E-06
            t_step_temp = 0;
        end
    end   
    
    t_step_horizon(:, i) = t_step_temp;
    
    t_step_temp = t_step_temp + PARA.dt_MPC;
end
end