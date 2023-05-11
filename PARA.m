classdef PARA < handle
    properties (Constant)  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        H = 20; % : Number of horizons
        dt_MPC = 0.025; % [s] : sampling time of MPC        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % World
        D2R = pi/180;
        R2D = 180/pi;
        g = 9.81;
        dt = 0.001;
        
        % Robot
        pelvis_width = 0.22;
        zc = 0.74;
        w = sqrt(PARA.g/PARA.zc);
        Foot_length = 0.18;
        Foot_width = 0.13;
        Foot_up_max = 0.12;
        V_x_max = 0.7; V_x_min = -0.7;
        V_y_max = 0.6; V_y_min = -0.6;
        J_x = 2;
        J_y = 2;
        kp = 6;
        kd = 4;
        
        % Preview control
        T_preview = 1.6;
        NL = PARA.T_preview/PARA.dt;
        A_preview = [1, PARA.dt, (PARA.dt*PARA.dt)/2;
                     0, 1, PARA.dt; 
                     0, 0, 1];
        B_preview = [(PARA.dt*PARA.dt*PARA.dt)/6; 
                     (PARA.dt*PARA.dt)/2; 
                      PARA.dt];
        C_preview = [1, 0, -(1/(PARA.w^2))];
        
        NL_MPC = PARA.T_preview/PARA.dt_MPC;
        A_preview_MPC = [1, PARA.dt_MPC, (PARA.dt_MPC*PARA.dt_MPC)/2;
                         0, 1, PARA.dt_MPC; 
                         0, 0, 1];
        B_preview_MPC = [(PARA.dt_MPC*PARA.dt_MPC*PARA.dt_MPC)/6; 
                         (PARA.dt_MPC*PARA.dt_MPC)/2; 
                         PARA.dt_MPC];
        C_preview_MPC = [1, 0, -(1/(PARA.w^2))];
        
        % NMPC
        T_scale = PARA.dt_MPC/PARA.dt;
        T_window = PARA.dt_MPC * PARA.H;
        N_window = PARA.T_window/PARA.dt;
        state_length = 2; % [xi_err_x; xi_err_y]
        input_length = 9; % [p_c_x; p_c_y; dU_x; dU_y; db_x; db_y; dT; ddtheta_x; ddtheta_y];   

        w_dT = 100;

        w_xi_err_x = 1;
        w_p_c_x = 10;
        w_dU_x = 1;
        w_db_x = 1000;
        w_ddtheta_y = 0.010;
        
        w_xi_err_y = 1;
        w_p_c_y = 50;
        w_dU_y = 100;
        w_db_y = 1000;
        w_ddtheta_x = 0.10; 

        p_c_x_max =  0.5*PARA.Foot_length;
        p_c_y_max =  0.5*PARA.Foot_width;
        p_c_x_min = -0.5*PARA.Foot_length;
        p_c_y_min = -0.5*PARA.Foot_width;

        dU_x_max =  0.3;
        dU_y_max =  0.2;
        dU_x_min = -0.3;
        dU_y_min = -0.2;
        dT_max = 0.0;
        dT_min = -0.2;
        
        theta_x_min = -40*PARA.D2R;        theta_x_max =  40*PARA.D2R;
        theta_y_min = -30*PARA.D2R;        theta_y_max =  15*PARA.D2R;
        dtheta_x_max = 200*PARA.D2R;
        dtheta_y_max = 200*PARA.D2R;
        ddtheta_x_max = 1000*PARA.D2R;
        ddtheta_y_max = 1000*PARA.D2R;
        ddtheta_x_min = -1000*PARA.D2R;
        ddtheta_y_min = -1000*PARA.D2R;

        % Kinematics - from Gazelle`s data
        m_PEL = 15;
        m_TORSO = 5;
        m_LHY = 0.01;
        m_LHR = 3;
        m_LHP = 6;
        m_LKN = 2;
        m_LAP = 0.01;
        m_LAR = 1;
        m_RHY = 0.01;
        m_RHR = 3;
        m_RHP = 6;
        m_RKN = 2;
        m_RAP = 0.01;
        m_RAR = 1;
        m_all = PARA.m_PEL + PARA.m_TORSO + (PARA.m_LHY+PARA.m_LHR+PARA.m_LHP+PARA.m_LKN+PARA.m_LAP+PARA.m_LAR) + (PARA.m_RHY+PARA.m_RHR+PARA.m_RHP+PARA.m_RKN+PARA.m_RAP+PARA.m_RAR);
        
        l0 = 0.15;
        l1 = 0.105;
        l2 = 0.05;
        l3 = 0.06;
        l4 = 0.4;
        l5 = 0.38;
        l6 = 0.07;
        
        c_PEL = [0; 0; 0];
        c_TORSO = [0; 0; 0];
        c_LHY = [0; 0; 0];
        c_LHR = [0; 0.5*PARA.l2; -0.5*PARA.l3];
        c_LHP = [0; 0; -0.5*PARA.l4];
        c_LKN = [0; 0; -0.5*PARA.l5];
        c_LAP = [0; 0; 0];
        c_LAR = [0; 0; -0.5*PARA.l6];
        c_RHY = [0; 0; 0];
        c_RHR = [0; -0.5*PARA.l2; -0.5*PARA.l3];
        c_RHP = [0; 0; -0.5*PARA.l4];
        c_RKN = [0; 0; -0.5*PARA.l5];
        c_RAP = [0; 0; 0];
        c_RAR = [0; 0; -0.5*PARA.l6];
    end
end