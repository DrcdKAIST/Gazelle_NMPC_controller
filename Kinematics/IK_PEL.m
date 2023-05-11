% Input: Kinematic information, x_target / Output: q_target
% x_target = [pPEL; qPEL; pLF; qLF; pRF; qRF];
function q_target = IK_PEL(x_target)    
    l1 = PARA.l1;
    l2 = PARA.l2;
    l3 = PARA.l3;
    l4 = PARA.l4;
    l5 = PARA.l5;
    l6 = PARA.l6;
    
    pPEL = x_target(1:3);
    qPEL = x_target(4:7);
    pLF  = x_target(8:10);
    qLF  = x_target(11:14);
    pRF  = x_target(15:17);
    qRF  = x_target(18:21);    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LLEG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % q6
    rotPEL = quat2rotm(qPEL');
    pLHR = pPEL + rotPEL*[0; l1; 0];
    rotLF = quat2rotm(qLF');
    LF2LHR = rotLF\(pLHR-pLF);
    LF2LAR = [0; 0; l6];
    LAR2LHR = LF2LHR - LF2LAR;
    %   tan(q6) = -(f_AR2HR.y + l2cos(q6))/(f_AR2HR.z + l2cos(q6))
    %   tan(q6/2) = t , cos(q6) = 2t/(1+t^2) , sin(q6) = (1 - t^2)/(1 + t^2)
    %   t  = (Z +- (Y^2 + Z^2 - l2^2)^(1/2))/(Y - l2)
    t = -(LAR2LHR(3)-sqrt(LAR2LHR(2)^2+LAR2LHR(3)^2-l2^2))/(LAR2LHR(2)-l2);
    %%%%%%%%%%%%%%%
    q6 = 2*atan(t);
    %%%%%%%%%%%%%%%
    
    % q1, q2
    PEL2LF = pLF - pPEL;
    rot0toF = rotPEL\rotLF;
    PEL2LF = rotPEL\PEL2LF;
    T0toF = [rot0toF PEL2LF; [0 0 0 1]];
    T6toF = [eye(3) [0; 0; -l6]; [0 0 0 1]];
    rot5to6 = rotX_rad(q6);
    T5to6 = [rot5to6 [0; 0; 0]; [0 0 0 1]];
    rot0to5 = rot0toF*rot5to6';
    T0to5 = T0toF*[eye(3) [0; 0; l6]; [0 0 0 1]]*[rotX_rad(q6).' [0; 0; 0]; [0 0 0 1]];
    %%%%%%%%%%%%%%%%%%%%%%
    q2 = asin(T0to5(3,2));  %T0to5(3,2) = sin(q2)
    %%%%%%%%%%%%%%%%%%%%%%
    sin_q1 = T0to5(1,2)/cos(q2); %T0to5(1,2) = -cos(q2)*sin(q1);
    cos_q1 = T0to5(2,2)/cos(q2); %T0to5(2,2) = cos(q1)*cos(q2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    q1 = atan2(sin_q1,cos_q1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % q345 = q3+q4+q5
    rot0to1 = rotZ_rad(q1);
    rot1to2 = rotX_rad(q2);
    rot2to5 = rot1to2'*rot0to1'*rot0to5;
    %rot2to5 = [  cos(q3 + q4 + q5), 0, sin(q3 + q4 + q5)]
    %          [                  0, 1,                 0]
    %          [ -sin(q3 + q4 + q5), 0, cos(q3 + q4 + q5)]
    cos_q345 = rot2to5(1,1);
    sin_q345 = rot2to5(1,3);
    q345 = atan2(sin_q345, cos_q345);
    
    % q4
    LHR2LHP_PEL_Frame = rot0to1*rot1to2*[0; l2; -l3];
    LHR2LHP_F_Frame = rot0toF'*LHR2LHP_PEL_Frame;
    LAP2LHP = LHR2LHP_F_Frame + LAR2LHR;
    LAP2LHP_length = norm(LAP2LHP);
    if LAP2LHP_length > (l4+l5-0.01)
        LAP2LHP_length = l4+l5-0.01;
    end
    cos_pi_minus_q4 = (l4^2+l5^2-LAP2LHP_length^2)/(2*l4*l5);
    if cos_pi_minus_q4 > 1
        cos_pi_minus_q4 = 1;
    elseif cos_pi_minus_q4 < -1
        cos_pi_minus_q4 = -1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q4 = (pi-acos(cos_pi_minus_q4));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % q3
    T2to5 = [rot1to2' [0; 0; 0]; [0 0 0 1]]*[rot0to1' -rot0to1'*[0; l1; 0]; [0 0 0 1]]*T0to5; %T2to5 = T1to2'*T0to1'*T0to5;
    % T2to5(1,4) = - l5*(cos(q3)*sin(q4) + cos(q4)*sin(q3)) - l4*sin(q3)
    % T2to5(3,4) = - l3 - l5*(cos(q3)*cos(q4) - sin(q3)*sin(q4)) - l4*cos(q3)
    A1 = -l5*sin(q4);
    A2 = -l5*cos(q4) - l4;
    A3 = T2to5(1,4);
    B1 = -l5*cos(q4) - l4;
    B2 = l5*sin(q4);
    B3 = l3 + T2to5(3,4);
    sin_q3 = (A3*B1 - B3*A1)/(A2*B1 - A1*B2);
    cos_q3 = (A3*B2 - B3*A2)/(A1*B2 - A2*B1);
    q3 = atan2(sin_q3, cos_q3);
    
    % q5
    q5 = q345 - q4 - q3;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RLEG %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % q12(q6)
    rotPEL = quat2rotm(qPEL');
    pRHR = pPEL + rotPEL*[0; -l1; 0];
    rotRF = quat2rotm(qRF');
    RF2RHR = rotRF\(pRHR-pRF);
    RF2RAR = [0; 0; l6];
    RAR2RHR = RF2RHR - RF2RAR;
    %   tan(q6) = -(f_AR2HR.y + l2cos(q6))/(f_AR2HR.z + l2cos(q6))
    %   tan(q6/2) = t , cos(q6) = 2t/(1+t^2) , sin(q6) = (1 - t^2)/(1 + t^2)
    %   t  = (Z +- (Y^2 + Z^2 - l2^2)^(1/2))/(Y - l2)
    t = -(RAR2RHR(3)-sqrt(RAR2RHR(2)^2+RAR2RHR(3)^2-l2^2))/(RAR2RHR(2)+l2);
    %%%%%%%%%%%%%%%
    q12 = 2*atan(t);
    %%%%%%%%%%%%%%%
    
    % q7(q1), q8(q2)
    PEL2RF = pRF - pPEL;
    rot0toF = rotPEL\rotRF;
    PEL2RF = rotPEL\PEL2RF;
    T0toF = [rot0toF PEL2RF; [0 0 0 1]];
    T6toF = [eye(3) [0; 0; -l6]; [0 0 0 1]];
    rot5to6 = rotX_rad(q12);
    T5to6 = [rot5to6 [0; 0; 0]; [0 0 0 1]];
    rot0to5 = rot0toF*rot5to6';
    T0to5 = T0toF*[eye(3) [0; 0; l6]; [0 0 0 1]]*[rotX_rad(q12).' [0; 0; 0]; [0 0 0 1]];
    %%%%%%%%%%%%%%%%%%%%%%
    q8 = asin(T0to5(3,2));  %T0to5(3,2) = sin(q2)
    %%%%%%%%%%%%%%%%%%%%%%
    sin_q7 = T0to5(1,2)/cos(q8); %T0to5(1,2) = -cos(q2)*sin(q1);
    cos_q7 = T0to5(2,2)/cos(q8); %T0to5(2,2) = cos(q1)*cos(q2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    q7 = atan2(sin_q7,cos_q7);
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % q91011(q345) = q9+q10+q11
    rot0to1 = rotZ_rad(q7);
    rot1to2 = rotX_rad(q8);
    rot2to5 = rot1to2'*rot0to1'*rot0to5;
    %rot2to5 = [  cos(q3 + q4 + q5), 0, sin(q3 + q4 + q5)]
    %          [                  0, 1,                 0]
    %          [ -sin(q3 + q4 + q5), 0, cos(q3 + q4 + q5)]
    cos_q91011 = rot2to5(1,1);
    sin_q91011 = rot2to5(1,3);
    q91011 = atan2(sin_q91011, cos_q91011);
    
    % q4(q10)
    RHR2RHP_PEL_Frame = rot0to1*rot1to2*[0; -l2; -l3];
    RHR2RHP_F_Frame = rot0toF'*RHR2RHP_PEL_Frame;
    RAP2RHP = RHR2RHP_F_Frame + RAR2RHR;
    RAP2RHP_length = norm(RAP2RHP);
    if RAP2RHP_length > (l4+l5-0.01)
        RAP2RHP_length = l4+l5-0.01;
    end
    cos_pi_minus_q10 = (l4^2+l5^2-RAP2RHP_length^2)/(2*l4*l5);
    if cos_pi_minus_q10 > 1
        cos_pi_minus_q10 = 1;
    elseif cos_pi_minus_q10 < -1
        cos_pi_minus_q10 = -1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q10 = (pi-acos(cos_pi_minus_q10));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % q9(q3)
    T2to5 = [rot1to2' [0; 0; 0]; [0 0 0 1]]*[rot0to1' -rot0to1'*[0; -l1; 0]; [0 0 0 1]]*T0to5; %T2to5 = T1to2'*T0to1'*T0to5;
    % T2to5(1,4) = - l5*(cos(q3)*sin(q4) + cos(q4)*sin(q3)) - l4*sin(q3)
    % T2to5(3,4) = - l3 - l5*(cos(q3)*cos(q4) - sin(q3)*sin(q4)) - l4*cos(q3)
    A1 = -l5*sin(q10);
    A2 = -l5*cos(q10) - l4;
    A3 = T2to5(1,4);
    B1 = -l5*cos(q10) - l4;
    B2 = l5*sin(q10);
    B3 = l3 + T2to5(3,4);
    sin_q3 = (A3*B1 - B3*A1)/(A2*B1 - A1*B2);
    cos_q3 = (A3*B2 - B3*A2)/(A1*B2 - A2*B1);
    q9 = atan2(sin_q3, cos_q3);
    
    % q5
    q11 = q91011 - q9 - q10;
    
    q_target = [q1; q2; q3; q4; q5; q6; q7; q8; q9; q10; q11; q12];
end