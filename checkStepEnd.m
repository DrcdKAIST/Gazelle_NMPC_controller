function flag_STEP_CHANGE = checkStepEnd(t_step, T_step)
if t_step + 0.5*PARA.dt > T_step
    flag_STEP_CHANGE = 1;
else
    flag_STEP_CHANGE = 0;
end
end
