%--- Path setting
clc; clear all; close all;
% restoredefaultpath;
folder = fileparts(which('main.m'));
addpath(genpath(folder));
%---

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step information
number_of_step = 8;             % Number of steps
step_length = 0.15;             % Step stride
step_width = PARA.pelvis_width; % Step width
step_time = 0.7;                % Step period
L_or_R = 1;                     % First swing foot: 1: Left foot / -1: Right foot

% Disturbance information
Impact_force_x = 475;             % [N]: x-dir impact force
Impact_force_y = 0;             % [N]: y-dir impact force
Impact_duration = 0.05;         % [s]: Impact duration
Impact_timing = 0.0;            % [s]: Timing of impact
Impact_step_number = 3;         
% Disturbance with [Impact_force_x; Impact_force_y] magnitude and
% [Impact_duration] duration is applied at [Impact_timing] [s] of
% [Impact_step_number]th step.

% Flags
flag_HORIZON_CHANGED = 0;       % Set to 1 if the number of MPC horizon is changed
flag_VISUALIZATION = 1;         % Set to 1 for graphic ON
flag_VISUALIZATION_ROBOT = 1;   % Set to 1 to show robot
flag_PLOT = 1;                  % Set to 1 to show plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--- Get gradients and hessians with CasADI
if flag_HORIZON_CHANGED == 1
    disp(['Now creating gradients and hessians with CasADI for the MPC horizon: ', num2str(PARA.H), '...']);
    getGradHessWithCasadi();
end
%---

%--- Global variables
global flag_EXIT flag_PAUSE;
global dx dy;
dx = 0; dy = 0;
%---

%--- SDB(Step Data Buffer)
p_ref_total = zmpTotal(number_of_step, step_length, step_width, L_or_R);
p_total = zmpTotal(number_of_step, step_length, step_width, L_or_R);

T_step_ref_total = stepTimeTotal(number_of_step, step_time);
T_step_total = stepTimeTotal(number_of_step, step_time);
%---

%--- Initialization
t = 0; t_step = 0;
i = 1; i_max = 1E06;
T_step = T_step_total(:, 1);
T_step_ref = T_step_ref_total(:, 1);
step_phase = 1;
iter_error = 0;
% Flags
flag_STEP_CHANGE = 0;
flag_EXIT = 0;
flag_PAUSE = 0;
flag_ERROR = 0;
% Preview control
[Gi, Gx, Gp] = findPreviewGain(PARA.T_preview, PARA.dt, PARA.zc);
[Gi_MPC, Gx_MPC, Gp_MPC] = findPreviewGain(PARA.T_preview, PARA.dt_MPC, PARA.zc);
p_err_sum_x = 0; p_err_sum_y = 0;
p_err_sum_x_ref = 0; p_err_sum_y_ref = 0;
% ZMP
p_des = p_total(:, 1);
% COM
COM = [0; 0; PARA.zc];
dCOM = [0; 0; 0];
COM_prev_step = COM;
dCOM_prev_step = dCOM;
xi_err = [0; 0];
% COM ref.
COM_ref = [0; 0; PARA.zc];
dCOM_ref = [0; 0; 0];
ddCOM_ref = [0; 0; 0];
% Foot
Foot_state = 2; % DSP
LF = [0;  0.5*PARA.pelvis_width; 0]; LF_prev = LF;
RF = [0; -0.5*PARA.pelvis_width; 0]; RF_prev = RF;
dU_x_prev = 0; dU_y_prev = 0;
color_LF = [0.9290 0.6940 0.1250];
color_RF = [0.6 0.6 0.6];
% Torso
theta = [0; 0];
dtheta = [0; 0];
%---

%--- Data save
t_stored = zeros(1, i_max);
t_step_stored = zeros(1, i_max);
impact_force_stored = zeros(2, i_max);
T_step_stored = zeros(1, i_max);
xi_err_stored = zeros(3, i_max);
xi_stored = zeros(3, i_max);
COM_stored = zeros(3, i_max);
dCOM_stored = zeros(3, i_max);
p_c_stored = zeros(3, i_max); 
dU_stored = zeros(2, i_max); 
db_stored = zeros(2, i_max); 
dT_stored = zeros(1, i_max); 
ddtheta_result_stored = zeros(2, i_max); 
LF_stored = zeros(3, i_max);
RF_stored = zeros(3, i_max);
ticktock_stored = zeros(1, i_max);
%---

%--- World generation
if ishandle(10)
    close(10)
end
if flag_VISUALIZATION == 1
    fig_world = figure(10);
    set(fig_world, 'Position', [1000 540 920 455], 'Renderer', 'OpenGL', 'Color',[1,1,1], 'KeyPressFcn', @printfig);
    axe = axes('Parent', fig_world);
    if flag_VISUALIZATION_ROBOT == 1
        ax = 55; ay = 15;
    else
        ax = 0; ay = 89.999;
    end
    view([ax ay]);
    set(axe, 'XLim', [-0.5 0.5], 'YLim', [-0.5 0.5], 'ZLim', [-0.02 1], 'DataAspectRatio', [1 1 1]);
    grid on; grid minor;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
end
%---

%--- Main Loop
while 1
    tic;
    if flag_VISUALIZATION == 1
        % Camera control
        ax = ax - dx;
        ay = ay - dy;
        dx = 0; dy = 0;
        view([ax, ay]);
        set(axe,'XLim',[-0.5+COM(1) 0.5+COM(1)],'YLim',[-0.5+COM(2) 0.5+COM(2)],'ZLim',[-0.02 1.0], 'DataAspectRatio', [1 1 1]);
%         set(axe,'XLim',[-0.5+COM(1) 0.5+COM(1)],'YLim',[-0.5 0.5],'ZLim',[-0.02 1.0], 'DataAspectRatio', [1 1 1]);
    end

    % Step change
    flag_STEP_CHANGE = checkStepEnd(t_step, T_step);
    if flag_STEP_CHANGE == 1
        % Pattern shifting
        for j = step_phase+1:number_of_step+3
            p_ref_total(1, j) = p_ref_total(1, j) + dU(1);
            p_ref_total(2, j) = p_ref_total(2, j) + dU(2);
        end   
        COM_ref = COM_ref + [dU(1); dU(2); 0];

        % Update step phase
        step_phase = step_phase + 1;
        if step_phase > number_of_step + 3
            flag_EXIT = 1;
        end

        % Update foot state
        Foot_state = (-1)*Foot_state;
        if step_phase == 2
            Foot_state = L_or_R;
        elseif step_phase == number_of_step + 3
            Foot_state = 2;
        end

        % Update step time
        if step_phase <= number_of_step + 3
            T_step_ref = T_step_ref_total(1, step_phase);
            T_step = T_step_total(:, step_phase);
        end

        % Reset t_step & flag
        t_step = 0;
        flag_STEP_CHANGE = 0; 
    end

    % Exit flag
    if (norm(xi_err) > 1.0)
        disp('Walking fail!!');
        break;
    elseif (flag_EXIT == 1)
        disp('Walking finish!!');
        close(10);
        break;
    end

    % Disturbance
    disturbance_duration = Impact_duration; % [sec]
    disturbance_timing = Impact_timing;
    if (step_phase == Impact_step_number + 1) && ((t_step >= disturbance_timing))
        flag_ERROR = 1;
    end
    if (flag_ERROR == 1) && (iter_error > (Impact_duration/PARA.dt))
        flag_ERROR = 0;
    end
    if flag_ERROR == 1
        disturbance_magnitude = [Impact_force_x; Impact_force_y; 0]; % [N]
        ddCOM_err = disturbance_magnitude/PARA.m_all;
        dCOM_err = ddCOM_err.*PARA.dt;
        COM_err = dCOM_err.*PARA.dt + 0.5.*ddCOM_err.*PARA.dt.*PARA.dt;
        COM = COM + COM_err;
        dCOM = dCOM + dCOM_err;
        iter_error = iter_error + 1;
    else     
        disturbance_magnitude = [0; 0; 0]; % [N]
        ddCOM_err = [0; 0; 0];
    end

    % Reference    
    [COM_ref_next, dCOM_ref_next, ddCOM_ref_next, p_err_sum_x_ref_next, p_err_sum_y_ref_next] = previewControl(t_step, step_phase, p_ref_total, T_step_ref_total, Gi, Gx, Gp, PARA.A_preview, PARA.B_preview, PARA.C_preview, COM_ref, dCOM_ref, ddCOM_ref, p_err_sum_x_ref, p_err_sum_y_ref);      
    xi_ref_horizon = dcmRefWindow(t_step, step_phase, COM_ref, dCOM_ref, ddCOM_ref, p_err_sum_x_ref, p_err_sum_y_ref, T_step_ref, p_ref_total, T_step_ref_total, Gi_MPC, Gx_MPC, Gp_MPC);

    % Get DCM error
    xi_ref = COM_ref + (dCOM_ref./PARA.w);
    xi = COM + (dCOM./PARA.w);
    xi_err = xi - xi_ref;

    % Calc control input
    T_step_ref_horizon = stepTimeRefHorizon(t_step, step_phase, T_step_ref_total);
    v0 = zeros(PARA.H*(PARA.state_length+PARA.input_length), 1);
    [p_c, dU, db, dT, ddtheta] = nextState(v0, t_step, xi_err, xi_ref_horizon, T_step_ref_horizon, dU_x_prev, dU_y_prev);

    for j = step_phase+1:number_of_step+3
        p_total(1, j) = p_ref_total(1, j) + dU(1);
        p_total(2, j) = p_ref_total(2, j) + dU(2);
    end    
    T_step = T_step_ref + dT;
    ddtheta_return_x = -PARA.kp*theta(1) - PARA.kd*dtheta(1);
    ddtheta_return_y = -PARA.kp*theta(2) - PARA.kd*dtheta(2);
    ddtheta_return = [ddtheta_return_x; ddtheta_return_y];
    ddtheta_x = ddtheta(1) + ddtheta_return_x;
    ddtheta_y = ddtheta(2) + ddtheta_return_y;
    [ddtheta_max_x, ddtheta_max_y, ddtheta_min_x, ddtheta_min_y] = ddthetaMaxMin(theta, dtheta);
    if ddtheta_x >= ddtheta_max_x; ddtheta_x = ddtheta_max_x; end
    if ddtheta_x <= ddtheta_min_x; ddtheta_x = ddtheta_min_x; end
    if ddtheta_y >= ddtheta_max_y; ddtheta_y = ddtheta_max_y; end
    if ddtheta_y <= ddtheta_min_y; ddtheta_y = ddtheta_min_y; end
    ddtheta_result = [ddtheta_x; ddtheta_y];  
    dtheta_next = dtheta + ddtheta_result.*PARA.dt;
    theta_next = theta + dtheta.*PARA.dt;

    % Resultance ZMP
    p_des = p_total(:, step_phase);
    p_c_ZMP = [p_c; 0];
    p_c_CMP = p_c_ZMP + [((PARA.J_y*ddtheta_result(2))/(PARA.m_all*PARA.g)); ((PARA.J_x*ddtheta_result(1))/(PARA.m_all*PARA.g)); 0];
    p_result = p_des + p_c_CMP;

    % Plant response
    t_span = [0 PARA.dt];
    y0 = [COM(1); COM(2); dCOM(1); dCOM(2)];
    [t_ode, y_ode] = ode45(@(t_ode, y_ode) odefunc(y_ode, p_result), t_span, y0);  
    COM_next = [y_ode(end, 1); y_ode(end, 2); PARA.zc];
    dCOM_next = [y_ode(end, 3); y_ode(end, 4); 0];

    % Foot trajectory
    if Foot_state == 2
        LF = LF_prev;
        RF = RF_prev;
    elseif Foot_state ==  1 % LF swing
        % LF
        LF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, T_step, p_total);
%         LF = p_total(:, step_phase+1);
        % RF
        RF = RF_prev;
    elseif Foot_state == -1 % RF swing
        % LF
        LF = LF_prev;
        % RF
        RF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, T_step, p_total);
%         RF = p_total(:, step_phase+1);
    end
    if (step_phase == number_of_step + 2)
        LF = LF_prev;
        RF = RF_prev;
    end
    
    % Time ticktock
    ticktock = toc;
    
    % Data save
    t_stored(:, i) = t;
    t_step_stored(:, i) = t_step;
    impact_force_stored(:, i) = [disturbance_magnitude(1); disturbance_magnitude(2)];
    T_step_stored(:, i) = T_step;
    xi_err_stored(:, i) = xi_err;
    xi_stored(:, i) = xi;
    COM_stored(:, i) = COM;
    dCOM_stored(:, i) = dCOM;
    p_c_stored(:, i) = p_c_ZMP;
    dU_stored(:, i) = dU;
    db_stored(:, i) = db;
    dT_stored(:, i) = dT;
    ddtheta_result_stored(:, i) = ddtheta_result;
    LF_stored(:, i) = LF;
    RF_stored(:, i) = RF;
    ticktock_stored(:, i) = ticktock;
    
    % Visualization
    if mod(i, 20) == 1  
        if flag_VISUALIZATION == 1
            if flag_VISUALIZATION_ROBOT == 1
                %--- Inverse kinematics - COM
                pCOM = COM; 
                qPEL = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); rotmPEL = quat2mat(qPEL);
                pLF = LF;   qLF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); % qLF = [1; 0; 0; 0];
                pRF = RF;   qRF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); % qRF = [1; 0; 0; 0];

                x_target = [pCOM; qPEL'; pLF; qLF'; pRF; qRF'];
                [q_target, pPEL] = IK_COM(x_target);
                q1 = q_target(1); q2 = q_target(2); q3 = q_target(3); q4  = q_target(4);  q5  = q_target(5);  q6  = q_target(6); % LLEG
                q7 = q_target(7); q8 = q_target(8); q9 = q_target(9); q10 = q_target(10); q11 = q_target(11); q12 = q_target(12); % RLEG
                %---

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

%                 link01_LLEG = cylinder(axe, [link01_LLEG_x; link01_LLEG_y; link01_LLEG_z]', 0.005, [0.6350, 0.0780, 0.1840]	, 1, 20);
                link01_LLEG = cylinder(axe, [link01_LLEG_x; link01_LLEG_y; link01_LLEG_z]', 0.005, [0 0 0], 1, 20);
    %             link12_LLEG = cylinder(axe, [link12_LLEG_x; link12_LLEG_y; link12_LLEG_z]', 0.005, [0 0 1], 1, 20);
%                 link23_LLEG = cylinder(axe, [link23_LLEG_x; link23_LLEG_y; link23_LLEG_z]', 0.005, [0.6350, 0.0780, 0.1840]	, 1, 20);
                link23_LLEG = cylinder(axe, [link23_LLEG_x; link23_LLEG_y; link23_LLEG_z]', 0.005, [0 0 0], 1, 20);
                link34_LLEG = cylinder(axe, [link34_LLEG_x; link34_LLEG_y; link34_LLEG_z]', 0.005, [0 0 0], 1, 20);
                link45_LLEG = cylinder(axe, [link45_LLEG_x; link45_LLEG_y; link45_LLEG_z]', 0.005, [0 0 0], 1, 20);
    %             link56_LLEG = cylinder(axe, [link56_LLEG_x; link56_LLEG_y; link56_LLEG_z]', 0.005, [0 0 1], 1, 20);
                link6e_LLEG = cylinder(axe, [link6e_LLEG_x; link6e_LLEG_y; link6e_LLEG_z]', 0.005, 'k', 1, 20);
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

%                 link01_RLEG = cylinder(axe, [link01_RLEG_x; link01_RLEG_y; link01_RLEG_z]', 0.005, [0.6350, 0.0780, 0.1840]	, 1, 20);
                link01_RLEG = cylinder(axe, [link01_RLEG_x; link01_RLEG_y; link01_RLEG_z]', 0.005, [0 0 0], 1, 20);
    %             link12_RLEG = cylinder(axe, [link12_RLEG_x; link12_RLEG_y; link12_RLEG_z]', 0.005, [1 0 0], 1, 20);
%                 link23_RLEG = cylinder(axe, [link23_RLEG_x; link23_RLEG_y; link23_RLEG_z]', 0.005, [0.6350, 0.0780, 0.1840]	, 1, 20);
                link23_RLEG = cylinder(axe, [link23_RLEG_x; link23_RLEG_y; link23_RLEG_z]', 0.005, [0 0 0], 1, 20);
                link34_RLEG = cylinder(axe, [link34_RLEG_x; link34_RLEG_y; link34_RLEG_z]', 0.005, [0 0 0], 1, 20);
                link45_RLEG = cylinder(axe, [link45_RLEG_x; link45_RLEG_y; link45_RLEG_z]', 0.005, [0 0 0], 1, 20);
    %             link56_RLEG = cylinder(axe, [link56_RLEG_x; link56_RLEG_y; link56_RLEG_z]', 0.005, [1 0 0], 1, 20);
                link6e_RLEG = cylinder(axe, [link6e_RLEG_x; link6e_RLEG_y; link6e_RLEG_z]', 0.005, 'k', 1, 20);
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

                link01_TORSO = cylinder(axe, [link01_TORSO_x; link01_TORSO_y; link01_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_1 = cylinder(axe, [link2_TORSO_x(1:2); link2_TORSO_y(1:2); link2_TORSO_z(1:2)]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_2 = cylinder(axe, [link2_TORSO_x(2:3); link2_TORSO_y(2:3); link2_TORSO_z(2:3)]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_3 = cylinder(axe, [link2_TORSO_x(3:4); link2_TORSO_y(3:4); link2_TORSO_z(3:4)]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_4 = cylinder(axe, [[link2_TORSO_x(4) link2_TORSO_x(1)]; [link2_TORSO_y(4) link2_TORSO_y(1)]; [link2_TORSO_z(4) link2_TORSO_z(1)]]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_1 = cylinder(axe, [link3_TORSO_x(1:2); link3_TORSO_y(1:2); link3_TORSO_z(1:2)]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_2 = cylinder(axe, [link3_TORSO_x(2:3); link3_TORSO_y(2:3); link3_TORSO_z(2:3)]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_3 = cylinder(axe, [link3_TORSO_x(3:4); link3_TORSO_y(3:4); link3_TORSO_z(3:4)]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_4 = cylinder(axe, [[link3_TORSO_x(4) link3_TORSO_x(1)]; [link3_TORSO_y(4) link3_TORSO_y(1)]; [link3_TORSO_z(4) link3_TORSO_z(1)]]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_1 = cylinder(axe, [link41_TORSO_x; link41_TORSO_y; link41_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_2 = cylinder(axe, [link42_TORSO_x; link42_TORSO_y; link42_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_3 = cylinder(axe, [link43_TORSO_x; link43_TORSO_y; link43_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_4 = cylinder(axe, [link44_TORSO_x; link44_TORSO_y; link44_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                %---
            end

            % COM
            visual_COM = animatedline('Marker', 'o', 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'black');
            addpoints(visual_COM, COM(1), COM(2), COM(3));
            % xi
            visual_xi = animatedline('Marker', 'o', 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black');
            addpoints(visual_xi, xi(1), xi(2), xi(3));

            % ZMP result
            visual_p_result = animatedline('Marker', 'o', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
            addpoints(visual_p_result, p_result(1), p_result(2), p_result(3));
            % LF
            visual_LF_p1 = LF + [ 0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_LF_p2 = LF + [-0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_LF_p3 = LF + [-0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_LF_p4 = LF + [ 0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_LF_x = [visual_LF_p1(1) visual_LF_p2(1) visual_LF_p3(1) visual_LF_p4(1) visual_LF_p1(1)];
            visual_LF_y = [visual_LF_p1(2) visual_LF_p2(2) visual_LF_p3(2) visual_LF_p4(2) visual_LF_p1(2)];
            visual_LF_z = [visual_LF_p1(3) visual_LF_p2(3) visual_LF_p3(3) visual_LF_p4(3) visual_LF_p1(3)];
            if (Foot_state == 1)
                visual_LF = animatedline(visual_LF_x, visual_LF_y, visual_LF_z, 'color', color_LF, 'LineWidth', 1.5);
                visual_LF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_LF_center, LF(1), LF(2), LF(3));
            elseif (Foot_state == -1)
                visual_LF = animatedline(visual_LF_x, visual_LF_y, visual_LF_z, 'color', color_LF, 'LineWidth', 1.5);
                visual_LF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_LF_center, LF(1), LF(2), LF(3));
            else
                visual_LF = animatedline(visual_LF_x, visual_LF_y, visual_LF_z, 'color', color_LF, 'LineWidth', 1.5);
                visual_LF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_LF_center, LF(1), LF(2), LF(3));
            end
            % RF
            visual_RF_p1 = RF + [ 0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_RF_p2 = RF + [-0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_RF_p3 = RF + [-0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_RF_p4 = RF + [ 0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_RF_x = [visual_RF_p1(1) visual_RF_p2(1) visual_RF_p3(1) visual_RF_p4(1) visual_RF_p1(1)];
            visual_RF_y = [visual_RF_p1(2) visual_RF_p2(2) visual_RF_p3(2) visual_RF_p4(2) visual_RF_p1(2)];
            visual_RF_z = [visual_RF_p1(3) visual_RF_p2(3) visual_RF_p3(3) visual_RF_p4(3) visual_RF_p1(3)];
            if (Foot_state == 1)
                visual_RF = animatedline(visual_RF_x, visual_RF_y, visual_RF_z, 'color', color_RF, 'LineWidth', 1.5);
                visual_RF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_RF_center, RF(1), RF(2), RF(3));       
            elseif (Foot_state == -1)
                visual_RF = animatedline(visual_RF_x, visual_RF_y, visual_RF_z, 'color', color_RF, 'LineWidth', 1.5);
                visual_RF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_RF_center, RF(1), RF(2), RF(3));       
            else
                visual_RF = animatedline(visual_RF_x, visual_RF_y, visual_RF_z, 'color', color_RF, 'LineWidth', 1.5);
                visual_RF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_RF_center, RF(1), RF(2), RF(3));           
            end           

            drawnow;
            if flag_PAUSE == 1
                disp('Walking pause!!');
                waitforbuttonpress;
                flag_PAUSE = 0;
            end

            delete(visual_COM);
            delete(visual_xi);
            delete(visual_p_result);
            delete(visual_LF); delete(visual_RF);
            delete(visual_LF_center); delete(visual_RF_center);
            if flag_VISUALIZATION_ROBOT == 1
                delete(link01_LLEG); delete(link23_LLEG); delete(link34_LLEG); delete(link45_LLEG); delete(link6e_LLEG); 
                delete(link01_RLEG); delete(link23_RLEG); delete(link34_RLEG); delete(link45_RLEG); delete(link6e_RLEG); 
                delete(link01_TORSO); 
                delete(link2_TORSO_1); delete(link2_TORSO_2); delete(link2_TORSO_3); delete(link2_TORSO_4); 
                delete(link3_TORSO_1); delete(link3_TORSO_2); delete(link3_TORSO_3); delete(link3_TORSO_4); 
                delete(link4_TORSO_1); delete(link4_TORSO_2); delete(link4_TORSO_3); delete(link4_TORSO_4); 
            end
        end
    end

    % Time waits for no one
    t = t + PARA.dt;
    t_step = t_step + PARA.dt;

    COM_ref = COM_ref_next; dCOM_ref = dCOM_ref_next; ddCOM_ref = ddCOM_ref_next;
    p_err_sum_x_ref = p_err_sum_x_ref_next; p_err_sum_y_ref = p_err_sum_y_ref_next;
    COM = COM_next; dCOM = dCOM_next;
    dU_x_prev = dU(1); dU_y_prev = dU(2);
    LF_prev = LF;
    RF_prev = RF;

    dtheta = dtheta_next;
    theta = theta_next;

    i = i + 1;
end
%---

%-- Plot
t_stored = t_stored(:, 2:i-1);
t_step_stored = t_step_stored(:, 2:i-1);
impact_force_stored = impact_force_stored(:, 2:i-1);
T_step_stored = T_step_stored(:, 2:i-1);
xi_err_stored = xi_err_stored(:, 2:i-1);
xi_stored = xi_stored(:, 2:i-1);
COM_stored = COM_stored(:, 2:i-1);
dCOM_stored = dCOM_stored(:, 2:i-1);
p_c_stored = p_c_stored(:, 2:i-1);
dU_stored = dU_stored(:, 2:i-1);
db_stored = db_stored(:, 2:i-1);
dT_stored = dT_stored(:, 2:i-1);
ddtheta_result_stored = ddtheta_result_stored(:, 2:i-1);
LF_stored = LF_stored(:, 2:i-1);
RF_stored = RF_stored(:, 2:i-1);
ticktock_stored = ticktock_stored(:, 2:i-1);
%%
if flag_PLOT == 1
    if ishandle(1)
        close(1)
    end
    fig1 = figure(1);
    set(fig1, 'Position', [100 100 600 800])
    tile = tiledlayout(5, 2); 
    tile.TileSpacing = 'compact';
    tile.Padding = 'compact';
    nexttile([1 2])
    subplot0 = plot(t_stored, impact_force_stored(1, :), 'color', 'k' , 'linewidth', 2.0);
    legend(subplot0, 'Disturbance', 'interpreter', 'latex', 'location', 'northeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[N]')
    xlabel('Time [s]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot1 = plot(t_stored, xi_err_stored(1, :), 'color', 'b' , 'linewidth', 2.0);
    legend(subplot1, '$\xi^{err}_{x}$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot2 = plot(t_stored, p_c_stored(1, :), '-', 'color', 'r', 'linewidth', 2.0);
    legend(subplot2, '$p_{\mathrm{c,ZMP},x}$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot3 = plot(t_stored, dU_stored(1, :), '-', 'color', [0, 0.5, 0], 'linewidth', 2.0);
    legend(subplot3, '$\delta u_{x}$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot4 = plot(t_stored, db_stored(1, :), '-', 'color', 'k', 'linewidth', 2.0);
    legend(subplot4, '$\delta b_{x}$', 'interpreter', 'latex', 'location', 'northeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot5 = plot(t_stored, dT_stored, '-', 'color', [0.75, 0, 0.75], 'linewidth', 2.0);
    legend(subplot5, '$\delta T$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[s]')
    xlabel('Time [s]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot6 = plot(t_stored, ddtheta_result_stored(2, :).*PARA.R2D, 'color', [0, 0.75, 0.75], 'linewidth', 2.0);
    legend(subplot6, '$\ddot{\theta}_{pitch}$', 'interpreter', 'latex', 'location', 'northeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[deg/s^2]')
    xlabel('Time [s]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 2])
    histogram1 = histogram(ticktock_stored.*1000);
    legend(histogram1, 'Solve time', 'interpreter', 'latex', 'location', 'northeast', 'Orientation', 'vertical', 'fontsize', 14);
    grid on; grid minor;
    xlim([0 10])
    ylabel('Frequency')
    xlabel('Solve time [ms]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;    
    
    if ishandle(2)
        close(2)
    end
    fig2 = figure(2);
    set(fig2, 'Position', [700 100 600 800])
    tile = tiledlayout(5, 2); 
    tile.TileSpacing = 'compact';
    tile.Padding = 'compact';
    nexttile([1 2])
    subplot0 = plot(t_stored, impact_force_stored(2, :), 'color', 'k' , 'linewidth', 2.0);
    legend(subplot0, 'Disturbance', 'interpreter', 'latex', 'location', 'northeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[N]')
    xlabel('Time [s]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot1 = plot(t_stored, xi_err_stored(2, :), 'color', 'b' , 'linewidth', 2.0);
    legend(subplot1, '$\xi^{err}_{y}$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot2 = plot(t_stored, p_c_stored(2, :), '-', 'color', 'r', 'linewidth', 2.0);
    legend(subplot2, '$p_{\mathrm{c,ZMP},y}$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot3 = plot(t_stored, dU_stored(2, :), '-', 'color', [0, 0.5, 0], 'linewidth', 2.0);
    legend(subplot3, '$\delta u_{y}$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot4 = plot(t_stored, db_stored(2, :), '-', 'color', 'k', 'linewidth', 2.0);
    legend(subplot4, '$\delta b_{y}$', 'interpreter', 'latex', 'location', 'northeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[m]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot5 = plot(t_stored, dT_stored, '-', 'color', [0.75, 0, 0.75], 'linewidth', 2.0);
    legend(subplot5, '$\delta T$', 'interpreter', 'latex', 'location', 'southeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[s]')
    xlabel('Time [s]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 1])
    subplot6 = plot(t_stored, ddtheta_result_stored(1, :).*PARA.R2D, 'color', [0, 0.75, 0.75], 'linewidth', 2.0);
    legend(subplot6, '$\ddot{\theta}_{roll}$', 'interpreter', 'latex', 'location', 'northeast', 'fontsize', 14)
    grid on; grid minor;
    ylabel('[deg/s^2]')
    xlabel('Time [s]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
    nexttile([1 2])
    histogram1 = histogram(ticktock_stored.*1000);
    legend(histogram1, 'Solve time', 'interpreter', 'latex', 'location', 'northeast', 'Orientation', 'vertical', 'fontsize', 14);
    grid on; grid minor;
    xlim([0 10])
    ylabel('Frequency')
    xlabel('Solve time [ms]')
    ax = gca;
    ax.XAxis.FontSize = 12;
    ax.YAxis.FontSize = 12;
end
%---

%--- Figure function
function printfig(~,evnt)
    global dx dy;
    global flag_EXIT flag_PAUSE;
    
    if double(evnt.Character) == 28
        dx = -5;
    elseif double(evnt.Character) == 29
        dx = 5;
    elseif double(evnt.Character) == 30
        dy = 5;
    elseif double(evnt.Character) == 31
        dy = -5;
    else
        dx = 0; dy = 0;
    end
    
    if evnt.Character == 'e'
        flag_EXIT = 1;
    elseif evnt.Character == 'f'
        flag_PAUSE = 1;        
    end
end