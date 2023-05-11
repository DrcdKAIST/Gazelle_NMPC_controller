function T_step_total = stepTimeTotal(number_of_step, T_step)
if number_of_step < 1
    disp('number_of_step should be positive!! (number_of_step < 1)');
    number_of_step = 1;
end

T_step_total = zeros(1, number_of_step + 3);

for i = 1:number_of_step+3
    if i == 1
        T_step_total(:, i) = T_step;
    elseif i == number_of_step + 3
        T_step_total(:, i) = 1.0*T_step;
    else
        T_step_total(:, i) = T_step;
    end
end
end