function dydt = odefunc(y_ode, p)
dydt = zeros(4, 1);

COM_x = y_ode(1);
COM_y = y_ode(2);
dCOM_x = y_ode(3);
dCOM_y = y_ode(4);
p_x = p(1);
p_y = p(2);

ddCOM_x = (PARA.w*PARA.w)*(COM_x - p_x);
ddCOM_y = (PARA.w*PARA.w)*(COM_y - p_y);

dydt(1) = dCOM_x;
dydt(2) = dCOM_y;
dydt(3) = ddCOM_x;
dydt(4) = ddCOM_y;
end