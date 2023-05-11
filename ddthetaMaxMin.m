function [ddtheta_max_x, ddtheta_max_y, ddtheta_min_x, ddtheta_min_y] = ddthetaMaxMin(theta_prev, dtheta_prev)

ddtheta_max_1_x = PARA.ddtheta_x_max;
ddtheta_max_1_y = PARA.ddtheta_y_max;
ddtheta_max_2_x = (PARA.dtheta_x_max - dtheta_prev(1))/PARA.dt;
ddtheta_max_2_y = (PARA.dtheta_y_max - dtheta_prev(2))/PARA.dt;
temp_1_theta_x = PARA.theta_x_max - theta_prev(1);
temp_1_theta_y = PARA.theta_y_max - theta_prev(2);
if temp_1_theta_x < 0; temp_1_theta_x = 0; end
if temp_1_theta_y < 0; temp_1_theta_y = 0; end
ddtheta_max_3_x = (sqrt(2*PARA.ddtheta_x_max*temp_1_theta_x)-dtheta_prev(1))/PARA.dt;
ddtheta_max_3_y = (sqrt(2*PARA.ddtheta_y_max*temp_1_theta_y)-dtheta_prev(2))/PARA.dt;
    
ddtheta_min_1_x = -PARA.ddtheta_x_max;
ddtheta_min_1_y = -PARA.ddtheta_y_max;
ddtheta_min_2_x = (-PARA.dtheta_x_max - dtheta_prev(1))/PARA.dt;
ddtheta_min_2_y = (-PARA.dtheta_y_max - dtheta_prev(2))/PARA.dt;
temp_2_theta_x = theta_prev(1) - PARA.theta_x_min;
temp_2_theta_y = theta_prev(2) - PARA.theta_y_min;
if temp_2_theta_x < 0; temp_2_theta_x = 0; end
if temp_2_theta_y < 0; temp_2_theta_y = 0; end
ddtheta_min_3_x = (-sqrt(2*PARA.ddtheta_x_max*temp_2_theta_x)-dtheta_prev(1))/PARA.dt;
ddtheta_min_3_y = (-sqrt(2*PARA.ddtheta_y_max*temp_2_theta_y)-dtheta_prev(2))/PARA.dt;

ddtheta_max_x = ddtheta_max_1_x;
if ddtheta_max_x >= ddtheta_max_2_x; ddtheta_max_x = ddtheta_max_2_x; end
if ddtheta_max_x >= ddtheta_max_3_x; ddtheta_max_x = ddtheta_max_3_x; end
ddtheta_max_y = ddtheta_max_1_y;
if ddtheta_max_y >= ddtheta_max_2_y; ddtheta_max_y = ddtheta_max_2_y; end
if ddtheta_max_y >= ddtheta_max_3_y; ddtheta_max_y = ddtheta_max_3_y; end

ddtheta_min_x = ddtheta_min_1_x;
if ddtheta_min_x <= ddtheta_min_2_x; ddtheta_min_x = ddtheta_min_2_x; end
if ddtheta_min_x <= ddtheta_min_3_x; ddtheta_min_x = ddtheta_min_3_x; end
ddtheta_min_y = ddtheta_min_1_y;
if ddtheta_min_y <= ddtheta_min_2_y; ddtheta_min_y = ddtheta_min_2_y; end
if ddtheta_min_y <= ddtheta_min_3_y; ddtheta_min_y = ddtheta_min_3_y; end

end