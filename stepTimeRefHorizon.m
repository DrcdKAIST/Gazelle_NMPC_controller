function T_step_ref_horizon = stepTimeRefHorizon(t_step, step_phase, T_step_ref_total)
H = PARA.H;

T_step_ref_horizon = zeros(1, H);

t_step_temp = t_step; step_phase_temp = step_phase;
T_step_ref_temp = T_step_ref_total(step_phase_temp);
for i = 1:H
    if t_step_temp > T_step_ref_temp
        if step_phase_temp + 1 <= length(T_step_ref_total)
            step_phase_temp = step_phase_temp + 1;
            T_step_ref_temp = T_step_ref_total(step_phase_temp);
        end
        
        t_step_temp = t_step_temp - T_step_ref_temp;
        if t_step_temp < 1E-06
            t_step_temp = 0;
        end
    end   
    
    T_step_ref_horizon(:, i) = T_step_ref_total(:, step_phase_temp);
    
    t_step_temp = t_step_temp + PARA.dt_MPC;
end
end