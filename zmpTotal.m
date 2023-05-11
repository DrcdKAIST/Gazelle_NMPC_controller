function p_total = zmpTotal(number_of_step, L_step, W_step, L_or_R)
if number_of_step < 1
    disp('number_of_step should be positive!! (number_of_step < 1)');
    number_of_step = 1;
end

p_total = zeros(3, number_of_step + 3);

for i = 1:number_of_step+3
    if i == 1
        p_total(:, i) = zeros(3, 1);
    elseif i == 2
        p_total(:, i) = p_total(:, i-1) + [0; L_or_R*0.5*PARA.pelvis_width; 0];
    elseif i == 3
        p_total(:, i) = p_total(:, i-1) + [0.5*L_step; L_or_R*W_step; 0];
    elseif i == number_of_step+2
        p_total(:, i) = p_total(:, i-1) + [0.5*L_step; L_or_R*W_step; 0];
    elseif i == number_of_step+3
        p_total(:, i) = p_total(:, i-1) + [0; L_or_R*0.5*PARA.pelvis_width; 0];
    else
        p_total(:, i) = p_total(:, i-1) + [L_step; L_or_R*W_step; 0];
    end
    L_or_R = (-1)*L_or_R;
end
end