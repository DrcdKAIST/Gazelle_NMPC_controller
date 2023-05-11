% function drawPointFootRobot(iter, iter_prev, pf_fig_handle, color_LF, color_RF, pf_COM_stored, pf_theta_stored, pf_LF_stored, pf_RF_stored)
function drawPointFootRobot(iter, pf_fig_handle, color_LF, color_RF, pf_COM_stored, pf_theta_stored, pf_LF_stored, pf_RF_stored)
% alpha = 0.7;
% color_prev = [0.7, 0.7, 0.7];
% 
% COM = pf_COM_stored(:, iter_prev);
% theta = pf_theta_stored(:, iter_prev);
% LF = pf_LF_stored(:, iter_prev);
% RF = pf_RF_stored(:, iter_prev);
% 
% pCOM = COM;
% qPEL = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); rotmPEL = quat2mat(qPEL);
% pLF = LF;   qLF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); % qLF = [1; 0; 0; 0];
% pRF = RF;   qRF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); % qRF = [1; 0; 0; 0];
% 
% x_target = [pCOM; qPEL'; pLF; qLF'; pRF; qRF'];
% [q_target, pPEL] = IK_COM_mex(x_target);
% %             [q_target, pPEL] = IK_COM(x_target);
% q1 = q_target(1); q2 = q_target(2); q3 = q_target(3); q4  = q_target(4);  q5  = q_target(5);  q6  = q_target(6); % LLEG
% q7 = q_target(7); q8 = q_target(8); q9 = q_target(9); q10 = q_target(10); q11 = q_target(11); q12 = q_target(12); % RLEG
% 
% %--- Leg
% T0 = [rotmPEL pPEL; [0 0 0 1]];
% % Left leg
% T1_LLEG =      T0*[rotZ_rad(q1) [0 PARA.l1 0]' ; [0 0 0 1]];
% T2_LLEG = T1_LLEG*[rotX_rad(q2) [0 0 0]' ; [0 0 0 1]];
% T3_LLEG = T2_LLEG*[rotY_rad(q3) [0 PARA.l2 -PARA.l3]' ; [0 0 0 1]];
% T4_LLEG = T3_LLEG*[rotY_rad(q4) [0 0 -PARA.l4]' ; [0 0 0 1]];
% T5_LLEG = T4_LLEG*[rotY_rad(q5) [0 0 -PARA.l5]' ; [0 0 0 1]];
% T6_LLEG = T5_LLEG*[rotX_rad(q6) [0 0 0]' ; [0 0 0 1]];
% Te_LLEG = T6_LLEG*[eye(3) [0 0 -PARA.l6]' ; [0 0 0 1]];
% 
% link01_LLEG_x = [T0(1,4) T1_LLEG(1,4)];    link01_LLEG_y = [T0(2,4) T1_LLEG(2,4)];   link01_LLEG_z = [T0(3,4) T1_LLEG(3,4)];
% link12_LLEG_x = [T1_LLEG(1,4) T2_LLEG(1,4)];	link12_LLEG_y = [T1_LLEG(2,4) T2_LLEG(2,4)];   link12_LLEG_z = [T1_LLEG(3,4) T2_LLEG(3,4)];
% link23_LLEG_x = [T2_LLEG(1,4) T3_LLEG(1,4)];	link23_LLEG_y = [T2_LLEG(2,4) T3_LLEG(2,4)];   link23_LLEG_z = [T2_LLEG(3,4) T3_LLEG(3,4)];
% link34_LLEG_x = [T3_LLEG(1,4) T4_LLEG(1,4)];	link34_LLEG_y = [T3_LLEG(2,4) T4_LLEG(2,4)];   link34_LLEG_z = [T3_LLEG(3,4) T4_LLEG(3,4)];
% link45_LLEG_x = [T4_LLEG(1,4) T5_LLEG(1,4)];	link45_LLEG_y = [T4_LLEG(2,4) T5_LLEG(2,4)];   link45_LLEG_z = [T4_LLEG(3,4) T5_LLEG(3,4)];
% link56_LLEG_x = [T5_LLEG(1,4) T6_LLEG(1,4)];	link56_LLEG_y = [T5_LLEG(2,4) T6_LLEG(2,4)];   link56_LLEG_z = [T5_LLEG(3,4) T6_LLEG(3,4)];
% link6e_LLEG_x = [T6_LLEG(1,4) Te_LLEG(1,4)];	link6e_LLEG_y = [T6_LLEG(2,4) Te_LLEG(2,4)];   link6e_LLEG_z = [T6_LLEG(3,4) Te_LLEG(3,4)];
% 
% link01_LLEG = cylinder(pf_fig_handle, [link01_LLEG_x; link01_LLEG_y; link01_LLEG_z]', 0.005, color_prev, alpha, 20);
% %             link12_LLEG = cylinder(axe, [link12_LLEG_x; link12_LLEG_y; link12_LLEG_z]', 0.005, [0 0 1], 1, 20);
% link23_LLEG = cylinder(pf_fig_handle, [link23_LLEG_x; link23_LLEG_y; link23_LLEG_z]', 0.005, color_prev, alpha, 20);
% link34_LLEG = cylinder(pf_fig_handle, [link34_LLEG_x; link34_LLEG_y; link34_LLEG_z]', 0.005, color_prev, alpha, 20);
% link45_LLEG = cylinder(pf_fig_handle, [link45_LLEG_x; link45_LLEG_y; link45_LLEG_z]', 0.005, color_prev, alpha, 20);
% %             link56_LLEG = cylinder(axe, [link56_LLEG_x; link56_LLEG_y; link56_LLEG_z]', 0.005, [0 0 1], 1, 20);
% link6e_LLEG = cylinder(pf_fig_handle, [link6e_LLEG_x; link6e_LLEG_y; link6e_LLEG_z]', 0.005, color_prev, alpha, 20);
% 
% % Right leg
% T1_RLEG =      T0*[rotZ_rad(q7) [0 -PARA.l1 0]' ; [0 0 0 1]];
% T2_RLEG = T1_RLEG*[rotX_rad(q8) [0 0 0]' ; [0 0 0 1]];
% T3_RLEG = T2_RLEG*[rotY_rad(q9) [0 -PARA.l2 -PARA.l3]' ; [0 0 0 1]];
% T4_RLEG = T3_RLEG*[rotY_rad(q10) [0 0 -PARA.l4]' ; [0 0 0 1]];
% T5_RLEG = T4_RLEG*[rotY_rad(q11) [0 0 -PARA.l5]' ; [0 0 0 1]];
% T6_RLEG = T5_RLEG*[rotX_rad(q12) [0 0 0]' ; [0 0 0 1]];
% Te_RLEG = T6_RLEG*[eye(3) [0 0 -PARA.l6]' ; [0 0 0 1]];
% 
% link01_RLEG_x = [T0(1,4) T1_RLEG(1,4)];    link01_RLEG_y = [T0(2,4) T1_RLEG(2,4)];   link01_RLEG_z = [T0(3,4) T1_RLEG(3,4)];
% link12_RLEG_x = [T1_RLEG(1,4) T2_RLEG(1,4)];	link12_RLEG_y = [T1_RLEG(2,4) T2_RLEG(2,4)];   link12_RLEG_z = [T1_RLEG(3,4) T2_RLEG(3,4)];
% link23_RLEG_x = [T2_RLEG(1,4) T3_RLEG(1,4)];	link23_RLEG_y = [T2_RLEG(2,4) T3_RLEG(2,4)];   link23_RLEG_z = [T2_RLEG(3,4) T3_RLEG(3,4)];
% link34_RLEG_x = [T3_RLEG(1,4) T4_RLEG(1,4)];	link34_RLEG_y = [T3_RLEG(2,4) T4_RLEG(2,4)];   link34_RLEG_z = [T3_RLEG(3,4) T4_RLEG(3,4)];
% link45_RLEG_x = [T4_RLEG(1,4) T5_RLEG(1,4)];	link45_RLEG_y = [T4_RLEG(2,4) T5_RLEG(2,4)];   link45_RLEG_z = [T4_RLEG(3,4) T5_RLEG(3,4)];
% link56_RLEG_x = [T5_RLEG(1,4) T6_RLEG(1,4)];	link56_RLEG_y = [T5_RLEG(2,4) T6_RLEG(2,4)];   link56_RLEG_z = [T5_RLEG(3,4) T6_RLEG(3,4)];
% link6e_RLEG_x = [T6_RLEG(1,4) Te_RLEG(1,4)];	link6e_RLEG_y = [T6_RLEG(2,4) Te_RLEG(2,4)];   link6e_RLEG_z = [T6_RLEG(3,4) Te_RLEG(3,4)];
% 
% link01_RLEG = cylinder(pf_fig_handle, [link01_RLEG_x; link01_RLEG_y; link01_RLEG_z]', 0.005, color_prev, alpha, 20);
% %             link12_RLEG = cylinder(axe, [link12_RLEG_x; link12_RLEG_y; link12_RLEG_z]', 0.005, [1 0 0], 1, 20);
% link23_RLEG = cylinder(pf_fig_handle, [link23_RLEG_x; link23_RLEG_y; link23_RLEG_z]', 0.005, color_prev, alpha, 20);
% link34_RLEG = cylinder(pf_fig_handle, [link34_RLEG_x; link34_RLEG_y; link34_RLEG_z]', 0.005, color_prev, alpha, 20);
% link45_RLEG = cylinder(pf_fig_handle, [link45_RLEG_x; link45_RLEG_y; link45_RLEG_z]', 0.005, color_prev, alpha, 20);
% %             link56_RLEG = cylinder(axe, [link56_RLEG_x; link56_RLEG_y; link56_RLEG_z]', 0.005, [1 0 0], 1, 20);
% link6e_RLEG = cylinder(pf_fig_handle, [link6e_RLEG_x; link6e_RLEG_y; link6e_RLEG_z]', 0.005, color_prev, alpha, 20);
% %---
% 
% %--- Torso
% TORSO_x = 0.12;   % [m]
% TORSO_y = 0.2;   % [m]
% TORSO_z = 0.22;   % [m]
% 
% T1_TORSO = T0*[eye(3) [0 0 PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
% T2_TORSO_1 = T0*[eye(3) [ 0.5*TORSO_x  0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
% T2_TORSO_2 = T0*[eye(3) [-0.5*TORSO_x  0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
% T2_TORSO_3 = T0*[eye(3) [-0.5*TORSO_x -0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
% T2_TORSO_4 = T0*[eye(3) [ 0.5*TORSO_x -0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
% T3_TORSO_1 = T0*[eye(3) [ 0.5*TORSO_x  0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
% T3_TORSO_2 = T0*[eye(3) [-0.5*TORSO_x  0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
% T3_TORSO_3 = T0*[eye(3) [-0.5*TORSO_x -0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
% T3_TORSO_4 = T0*[eye(3) [ 0.5*TORSO_x -0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
% 
% link01_TORSO_x = [T0(1,4) T1_TORSO(1,4)];    link01_TORSO_y = [T0(2,4) T1_TORSO(2,4)];   link01_TORSO_z = [T0(3,4) T1_TORSO(3,4)];
% link2_TORSO_x = [T2_TORSO_1(1,4) T2_TORSO_2(1,4) T2_TORSO_3(1,4) T2_TORSO_4(1,4) T2_TORSO_1(1,4)];
% link2_TORSO_y = [T2_TORSO_1(2,4) T2_TORSO_2(2,4) T2_TORSO_3(2,4) T2_TORSO_4(2,4) T2_TORSO_1(2,4)];
% link2_TORSO_z = [T2_TORSO_1(3,4) T2_TORSO_2(3,4) T2_TORSO_3(3,4) T2_TORSO_4(3,4) T2_TORSO_1(3,4)];
% link3_TORSO_x = [T3_TORSO_1(1,4) T3_TORSO_2(1,4) T3_TORSO_3(1,4) T3_TORSO_4(1,4) T3_TORSO_1(1,4)];
% link3_TORSO_y = [T3_TORSO_1(2,4) T3_TORSO_2(2,4) T3_TORSO_3(2,4) T3_TORSO_4(2,4) T3_TORSO_1(2,4)];
% link3_TORSO_z = [T3_TORSO_1(3,4) T3_TORSO_2(3,4) T3_TORSO_3(3,4) T3_TORSO_4(3,4) T3_TORSO_1(3,4)];
% link41_TORSO_x = [T2_TORSO_1(1,4) T3_TORSO_1(1,4)]; link41_TORSO_y = [T2_TORSO_1(2,4) T3_TORSO_1(2,4)]; link41_TORSO_z = [T2_TORSO_1(3,4) T3_TORSO_1(3,4)];
% link42_TORSO_x = [T2_TORSO_2(1,4) T3_TORSO_2(1,4)]; link42_TORSO_y = [T2_TORSO_2(2,4) T3_TORSO_2(2,4)]; link42_TORSO_z = [T2_TORSO_2(3,4) T3_TORSO_2(3,4)];
% link43_TORSO_x = [T2_TORSO_3(1,4) T3_TORSO_3(1,4)]; link43_TORSO_y = [T2_TORSO_3(2,4) T3_TORSO_3(2,4)]; link43_TORSO_z = [T2_TORSO_3(3,4) T3_TORSO_3(3,4)];
% link44_TORSO_x = [T2_TORSO_4(1,4) T3_TORSO_4(1,4)]; link44_TORSO_y = [T2_TORSO_4(2,4) T3_TORSO_4(2,4)]; link44_TORSO_z = [T2_TORSO_4(3,4) T3_TORSO_4(3,4)];
% 
% link01_TORSO = cylinder(pf_fig_handle, [link01_TORSO_x; link01_TORSO_y; link01_TORSO_z]', 0.005, color_prev, alpha, 20);
% link2_TORSO_1 = cylinder(pf_fig_handle, [link2_TORSO_x(1:2); link2_TORSO_y(1:2); link2_TORSO_z(1:2)]', 0.005, color_prev, alpha, 20);
% link2_TORSO_2 = cylinder(pf_fig_handle, [link2_TORSO_x(2:3); link2_TORSO_y(2:3); link2_TORSO_z(2:3)]', 0.005, color_prev, alpha, 20);
% link2_TORSO_3 = cylinder(pf_fig_handle, [link2_TORSO_x(3:4); link2_TORSO_y(3:4); link2_TORSO_z(3:4)]', 0.005, color_prev, alpha, 20);
% link2_TORSO_4 = cylinder(pf_fig_handle, [[link2_TORSO_x(4) link2_TORSO_x(1)]; [link2_TORSO_y(4) link2_TORSO_y(1)]; [link2_TORSO_z(4) link2_TORSO_z(1)]]', 0.005, color_prev, alpha, 20);
% link3_TORSO_1 = cylinder(pf_fig_handle, [link3_TORSO_x(1:2); link3_TORSO_y(1:2); link3_TORSO_z(1:2)]', 0.005, color_prev, alpha, 20);
% link3_TORSO_2 = cylinder(pf_fig_handle, [link3_TORSO_x(2:3); link3_TORSO_y(2:3); link3_TORSO_z(2:3)]', 0.005, color_prev, alpha, 20);
% link3_TORSO_3 = cylinder(pf_fig_handle, [link3_TORSO_x(3:4); link3_TORSO_y(3:4); link3_TORSO_z(3:4)]', 0.005, color_prev, alpha, 20);
% link3_TORSO_4 = cylinder(pf_fig_handle, [[link3_TORSO_x(4) link3_TORSO_x(1)]; [link3_TORSO_y(4) link3_TORSO_y(1)]; [link3_TORSO_z(4) link3_TORSO_z(1)]]', 0.005, color_prev, alpha, 20);
% link4_TORSO_1 = cylinder(pf_fig_handle, [link41_TORSO_x; link41_TORSO_y; link41_TORSO_z]', 0.005, color_prev, alpha, 20);
% link4_TORSO_2 = cylinder(pf_fig_handle, [link42_TORSO_x; link42_TORSO_y; link42_TORSO_z]', 0.005, color_prev, alpha, 20);
% link4_TORSO_3 = cylinder(pf_fig_handle, [link43_TORSO_x; link43_TORSO_y; link43_TORSO_z]', 0.005, color_prev, alpha, 20);
% link4_TORSO_4 = cylinder(pf_fig_handle, [link44_TORSO_x; link44_TORSO_y; link44_TORSO_z]', 0.005, color_prev, alpha, 20);
% %---
% 
% % LF
% visual_LF_center = animatedline(pf_fig_handle, 'Marker', 'o', 'MarkerFaceColor', color_prev, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
% addpoints(visual_LF_center, LF(1), LF(2), LF(3));
% 
% % RF
% visual_RF_center = animatedline(pf_fig_handle, 'Marker', 'o', 'MarkerFaceColor', color_prev, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
% addpoints(visual_RF_center, RF(1), RF(2), RF(3));      

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

COM = pf_COM_stored(:, iter);
theta = pf_theta_stored(:, iter);
LF = pf_LF_stored(:, iter);
RF = pf_RF_stored(:, iter);

pCOM = COM;
qPEL = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); rotmPEL = quat2mat(qPEL);
pLF = LF;   qLF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); % qLF = [1; 0; 0; 0];
pRF = RF;   qRF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); % qRF = [1; 0; 0; 0];

x_target = [pCOM; qPEL'; pLF; qLF'; pRF; qRF'];
[q_target, pPEL] = IK_COM_mex(x_target);
%             [q_target, pPEL] = IK_COM(x_target);
q1 = q_target(1); q2 = q_target(2); q3 = q_target(3); q4  = q_target(4);  q5  = q_target(5);  q6  = q_target(6); % LLEG
q7 = q_target(7); q8 = q_target(8); q9 = q_target(9); q10 = q_target(10); q11 = q_target(11); q12 = q_target(12); % RLEG

%--- Leg
T0 = [rotmPEL pPEL; [0 0 0 1]];
% Left leg
T1_LLEG =      T0*[rotZ_rad(q1) [0 PARA.l1 0]' ; [0 0 0 1]];
T2_LLEG = T1_LLEG*[rotX_rad(q2) [0 0 0]' ; [0 0 0 1]];
T3_LLEG = T2_LLEG*[rotY_rad(q3) [0 PARA.l2 -PARA.l3]' ; [0 0 0 1]];
T4_LLEG = T3_LLEG*[rotY_rad(q4) [0 0 -PARA.l4]' ; [0 0 0 1]];
T5_LLEG = T4_LLEG*[rotY_rad(q5) [0 0 -PARA.l5]' ; [0 0 0 1]];
T6_LLEG = T5_LLEG*[rotX_rad(q6) [0 0 0]' ; [0 0 0 1]];
Te_LLEG = T6_LLEG*[eye(3) [0 0 -PARA.l6]' ; [0 0 0 1]];

link01_LLEG_x = [T0(1,4) T1_LLEG(1,4)];    link01_LLEG_y = [T0(2,4) T1_LLEG(2,4)];   link01_LLEG_z = [T0(3,4) T1_LLEG(3,4)];
link12_LLEG_x = [T1_LLEG(1,4) T2_LLEG(1,4)];	link12_LLEG_y = [T1_LLEG(2,4) T2_LLEG(2,4)];   link12_LLEG_z = [T1_LLEG(3,4) T2_LLEG(3,4)];
link23_LLEG_x = [T2_LLEG(1,4) T3_LLEG(1,4)];	link23_LLEG_y = [T2_LLEG(2,4) T3_LLEG(2,4)];   link23_LLEG_z = [T2_LLEG(3,4) T3_LLEG(3,4)];
link34_LLEG_x = [T3_LLEG(1,4) T4_LLEG(1,4)];	link34_LLEG_y = [T3_LLEG(2,4) T4_LLEG(2,4)];   link34_LLEG_z = [T3_LLEG(3,4) T4_LLEG(3,4)];
link45_LLEG_x = [T4_LLEG(1,4) T5_LLEG(1,4)];	link45_LLEG_y = [T4_LLEG(2,4) T5_LLEG(2,4)];   link45_LLEG_z = [T4_LLEG(3,4) T5_LLEG(3,4)];
link56_LLEG_x = [T5_LLEG(1,4) T6_LLEG(1,4)];	link56_LLEG_y = [T5_LLEG(2,4) T6_LLEG(2,4)];   link56_LLEG_z = [T5_LLEG(3,4) T6_LLEG(3,4)];
link6e_LLEG_x = [T6_LLEG(1,4) Te_LLEG(1,4)];	link6e_LLEG_y = [T6_LLEG(2,4) Te_LLEG(2,4)];   link6e_LLEG_z = [T6_LLEG(3,4) Te_LLEG(3,4)];

link01_LLEG = cylinder(pf_fig_handle, [link01_LLEG_x; link01_LLEG_y; link01_LLEG_z]', 0.005, 'k', 1, 20);
%             link12_LLEG = cylinder(axe, [link12_LLEG_x; link12_LLEG_y; link12_LLEG_z]', 0.005, [0 0 1], 1, 20);
link23_LLEG = cylinder(pf_fig_handle, [link23_LLEG_x; link23_LLEG_y; link23_LLEG_z]', 0.005, 'k', 1, 20);
link34_LLEG = cylinder(pf_fig_handle, [link34_LLEG_x; link34_LLEG_y; link34_LLEG_z]', 0.005, 'k', 1, 20);
link45_LLEG = cylinder(pf_fig_handle, [link45_LLEG_x; link45_LLEG_y; link45_LLEG_z]', 0.005, 'k', 1, 20);
%             link56_LLEG = cylinder(axe, [link56_LLEG_x; link56_LLEG_y; link56_LLEG_z]', 0.005, [0 0 1], 1, 20);
link6e_LLEG = cylinder(pf_fig_handle, [link6e_LLEG_x; link6e_LLEG_y; link6e_LLEG_z]', 0.005, 'k', 1, 20);

% Right leg
T1_RLEG =      T0*[rotZ_rad(q7) [0 -PARA.l1 0]' ; [0 0 0 1]];
T2_RLEG = T1_RLEG*[rotX_rad(q8) [0 0 0]' ; [0 0 0 1]];
T3_RLEG = T2_RLEG*[rotY_rad(q9) [0 -PARA.l2 -PARA.l3]' ; [0 0 0 1]];
T4_RLEG = T3_RLEG*[rotY_rad(q10) [0 0 -PARA.l4]' ; [0 0 0 1]];
T5_RLEG = T4_RLEG*[rotY_rad(q11) [0 0 -PARA.l5]' ; [0 0 0 1]];
T6_RLEG = T5_RLEG*[rotX_rad(q12) [0 0 0]' ; [0 0 0 1]];
Te_RLEG = T6_RLEG*[eye(3) [0 0 -PARA.l6]' ; [0 0 0 1]];

link01_RLEG_x = [T0(1,4) T1_RLEG(1,4)];    link01_RLEG_y = [T0(2,4) T1_RLEG(2,4)];   link01_RLEG_z = [T0(3,4) T1_RLEG(3,4)];
link12_RLEG_x = [T1_RLEG(1,4) T2_RLEG(1,4)];	link12_RLEG_y = [T1_RLEG(2,4) T2_RLEG(2,4)];   link12_RLEG_z = [T1_RLEG(3,4) T2_RLEG(3,4)];
link23_RLEG_x = [T2_RLEG(1,4) T3_RLEG(1,4)];	link23_RLEG_y = [T2_RLEG(2,4) T3_RLEG(2,4)];   link23_RLEG_z = [T2_RLEG(3,4) T3_RLEG(3,4)];
link34_RLEG_x = [T3_RLEG(1,4) T4_RLEG(1,4)];	link34_RLEG_y = [T3_RLEG(2,4) T4_RLEG(2,4)];   link34_RLEG_z = [T3_RLEG(3,4) T4_RLEG(3,4)];
link45_RLEG_x = [T4_RLEG(1,4) T5_RLEG(1,4)];	link45_RLEG_y = [T4_RLEG(2,4) T5_RLEG(2,4)];   link45_RLEG_z = [T4_RLEG(3,4) T5_RLEG(3,4)];
link56_RLEG_x = [T5_RLEG(1,4) T6_RLEG(1,4)];	link56_RLEG_y = [T5_RLEG(2,4) T6_RLEG(2,4)];   link56_RLEG_z = [T5_RLEG(3,4) T6_RLEG(3,4)];
link6e_RLEG_x = [T6_RLEG(1,4) Te_RLEG(1,4)];	link6e_RLEG_y = [T6_RLEG(2,4) Te_RLEG(2,4)];   link6e_RLEG_z = [T6_RLEG(3,4) Te_RLEG(3,4)];

link01_RLEG = cylinder(pf_fig_handle, [link01_RLEG_x; link01_RLEG_y; link01_RLEG_z]', 0.005, 'k', 1, 20);
%             link12_RLEG = cylinder(axe, [link12_RLEG_x; link12_RLEG_y; link12_RLEG_z]', 0.005, [1 0 0], 1, 20);
link23_RLEG = cylinder(pf_fig_handle, [link23_RLEG_x; link23_RLEG_y; link23_RLEG_z]', 0.005, 'k', 1, 20);
link34_RLEG = cylinder(pf_fig_handle, [link34_RLEG_x; link34_RLEG_y; link34_RLEG_z]', 0.005, 'k', 1, 20);
link45_RLEG = cylinder(pf_fig_handle, [link45_RLEG_x; link45_RLEG_y; link45_RLEG_z]', 0.005, 'k', 1, 20);
%             link56_RLEG = cylinder(axe, [link56_RLEG_x; link56_RLEG_y; link56_RLEG_z]', 0.005, [1 0 0], 1, 20);
link6e_RLEG = cylinder(pf_fig_handle, [link6e_RLEG_x; link6e_RLEG_y; link6e_RLEG_z]', 0.005, 'k', 1, 20);
%---

%--- Torso
TORSO_x = 0.12;   % [m]
TORSO_y = 0.2;   % [m]
TORSO_z = 0.22;   % [m]

T1_TORSO = T0*[eye(3) [0 0 PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
T2_TORSO_1 = T0*[eye(3) [ 0.5*TORSO_x  0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
T2_TORSO_2 = T0*[eye(3) [-0.5*TORSO_x  0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
T2_TORSO_3 = T0*[eye(3) [-0.5*TORSO_x -0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
T2_TORSO_4 = T0*[eye(3) [ 0.5*TORSO_x -0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
T3_TORSO_1 = T0*[eye(3) [ 0.5*TORSO_x  0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
T3_TORSO_2 = T0*[eye(3) [-0.5*TORSO_x  0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
T3_TORSO_3 = T0*[eye(3) [-0.5*TORSO_x -0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
T3_TORSO_4 = T0*[eye(3) [ 0.5*TORSO_x -0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];

link01_TORSO_x = [T0(1,4) T1_TORSO(1,4)];    link01_TORSO_y = [T0(2,4) T1_TORSO(2,4)];   link01_TORSO_z = [T0(3,4) T1_TORSO(3,4)];
link2_TORSO_x = [T2_TORSO_1(1,4) T2_TORSO_2(1,4) T2_TORSO_3(1,4) T2_TORSO_4(1,4) T2_TORSO_1(1,4)];
link2_TORSO_y = [T2_TORSO_1(2,4) T2_TORSO_2(2,4) T2_TORSO_3(2,4) T2_TORSO_4(2,4) T2_TORSO_1(2,4)];
link2_TORSO_z = [T2_TORSO_1(3,4) T2_TORSO_2(3,4) T2_TORSO_3(3,4) T2_TORSO_4(3,4) T2_TORSO_1(3,4)];
link3_TORSO_x = [T3_TORSO_1(1,4) T3_TORSO_2(1,4) T3_TORSO_3(1,4) T3_TORSO_4(1,4) T3_TORSO_1(1,4)];
link3_TORSO_y = [T3_TORSO_1(2,4) T3_TORSO_2(2,4) T3_TORSO_3(2,4) T3_TORSO_4(2,4) T3_TORSO_1(2,4)];
link3_TORSO_z = [T3_TORSO_1(3,4) T3_TORSO_2(3,4) T3_TORSO_3(3,4) T3_TORSO_4(3,4) T3_TORSO_1(3,4)];
link41_TORSO_x = [T2_TORSO_1(1,4) T3_TORSO_1(1,4)]; link41_TORSO_y = [T2_TORSO_1(2,4) T3_TORSO_1(2,4)]; link41_TORSO_z = [T2_TORSO_1(3,4) T3_TORSO_1(3,4)];
link42_TORSO_x = [T2_TORSO_2(1,4) T3_TORSO_2(1,4)]; link42_TORSO_y = [T2_TORSO_2(2,4) T3_TORSO_2(2,4)]; link42_TORSO_z = [T2_TORSO_2(3,4) T3_TORSO_2(3,4)];
link43_TORSO_x = [T2_TORSO_3(1,4) T3_TORSO_3(1,4)]; link43_TORSO_y = [T2_TORSO_3(2,4) T3_TORSO_3(2,4)]; link43_TORSO_z = [T2_TORSO_3(3,4) T3_TORSO_3(3,4)];
link44_TORSO_x = [T2_TORSO_4(1,4) T3_TORSO_4(1,4)]; link44_TORSO_y = [T2_TORSO_4(2,4) T3_TORSO_4(2,4)]; link44_TORSO_z = [T2_TORSO_4(3,4) T3_TORSO_4(3,4)];

link01_TORSO = cylinder(pf_fig_handle, [link01_TORSO_x; link01_TORSO_y; link01_TORSO_z]', 0.005, 'k', 1, 20);
link2_TORSO_1 = cylinder(pf_fig_handle, [link2_TORSO_x(1:2); link2_TORSO_y(1:2); link2_TORSO_z(1:2)]', 0.005, 'k', 1, 20);
link2_TORSO_2 = cylinder(pf_fig_handle, [link2_TORSO_x(2:3); link2_TORSO_y(2:3); link2_TORSO_z(2:3)]', 0.005, 'k', 1, 20);
link2_TORSO_3 = cylinder(pf_fig_handle, [link2_TORSO_x(3:4); link2_TORSO_y(3:4); link2_TORSO_z(3:4)]', 0.005, 'k', 1, 20);
link2_TORSO_4 = cylinder(pf_fig_handle, [[link2_TORSO_x(4) link2_TORSO_x(1)]; [link2_TORSO_y(4) link2_TORSO_y(1)]; [link2_TORSO_z(4) link2_TORSO_z(1)]]', 0.005, 'k', 1, 20);
link3_TORSO_1 = cylinder(pf_fig_handle, [link3_TORSO_x(1:2); link3_TORSO_y(1:2); link3_TORSO_z(1:2)]', 0.005, 'k', 1, 20);
link3_TORSO_2 = cylinder(pf_fig_handle, [link3_TORSO_x(2:3); link3_TORSO_y(2:3); link3_TORSO_z(2:3)]', 0.005, 'k', 1, 20);
link3_TORSO_3 = cylinder(pf_fig_handle, [link3_TORSO_x(3:4); link3_TORSO_y(3:4); link3_TORSO_z(3:4)]', 0.005, 'k', 1, 20);
link3_TORSO_4 = cylinder(pf_fig_handle, [[link3_TORSO_x(4) link3_TORSO_x(1)]; [link3_TORSO_y(4) link3_TORSO_y(1)]; [link3_TORSO_z(4) link3_TORSO_z(1)]]', 0.005, 'k', 1, 20);
link4_TORSO_1 = cylinder(pf_fig_handle, [link41_TORSO_x; link41_TORSO_y; link41_TORSO_z]', 0.005, 'k', 1, 20);
link4_TORSO_2 = cylinder(pf_fig_handle, [link42_TORSO_x; link42_TORSO_y; link42_TORSO_z]', 0.005, 'k', 1, 20);
link4_TORSO_3 = cylinder(pf_fig_handle, [link43_TORSO_x; link43_TORSO_y; link43_TORSO_z]', 0.005, 'k', 1, 20);
link4_TORSO_4 = cylinder(pf_fig_handle, [link44_TORSO_x; link44_TORSO_y; link44_TORSO_z]', 0.005, 'k', 1, 20);
%---

% LF
visual_LF_center = animatedline(pf_fig_handle, 'Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
addpoints(visual_LF_center, LF(1), LF(2), LF(3));

% RF
visual_RF_center = animatedline(pf_fig_handle, 'Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
addpoints(visual_RF_center, RF(1), RF(2), RF(3));     
end