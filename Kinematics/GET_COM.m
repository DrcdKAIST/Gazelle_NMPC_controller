function COM = GET_COM(pPEL, qPEL, q_target)
    rotPEL = quat2rotm(qPEL');
    
    l0 = PARA.l0;
    l1 = PARA.l1;
    l2 = PARA.l2;
    l3 = PARA.l3;
    l4 = PARA.l4;
    l5 = PARA.l5;
    l6 = PARA.l6;

    q1 = q_target(1); q2 = q_target(2); q3 = q_target(3); q4  = q_target(4);  q5  = q_target(5);  q6  = q_target(6);  % LLEG
    q7 = q_target(7); q8 = q_target(8); q9 = q_target(9); q10 = q_target(10); q11 = q_target(11); q12 = q_target(12); % RLEG
    
    m_PEL = PARA.m_PEL;
  m_TORSO = PARA.m_TORSO;
    m_LHY = PARA.m_LHY;     m_RHY = PARA.m_RHY;
    m_LHR = PARA.m_LHR;     m_RHR = PARA.m_RHR;
    m_LHP = PARA.m_LHP;     m_RHP = PARA.m_RHP;
    m_LKN = PARA.m_LKN;     m_RKN = PARA.m_RKN;
    m_LAP = PARA.m_LAP;     m_RAP = PARA.m_RAP;
    m_LAR = PARA.m_LAR;     m_RAR = PARA.m_RAR;
    m_LLEG = m_LHY + m_LHR + m_LHP + m_LKN + m_LAP + m_LAR;
    m_RLEG = m_RHY + m_RHR + m_RHP + m_RKN + m_RAP + m_RAR;
    m_all = m_PEL + m_TORSO + m_LLEG + m_RLEG;
        
    c_PEL = PARA.c_PEL;
  c_TORSO = PARA.c_TORSO;
    c_LHY = PARA.c_LHY;     c_RHY = PARA.c_RHY;
    c_LHR = PARA.c_LHR;     c_RHR = PARA.c_RHR;
    c_LHP = PARA.c_LHP;     c_RHP = PARA.c_RHP;
    c_LKN = PARA.c_LKN;     c_RKN = PARA.c_RKN;
    c_LAP = PARA.c_LAP;     c_RAP = PARA.c_RAP;
    c_LAR = PARA.c_LAR;     c_RAR = PARA.c_RAR;
      
    T_PEL =       [rotPEL pPEL; [0 0 0 1]];
  T_TORSO = T_PEL*[eye(3) [0 0 l0]'; [0 0 0 1]];
    % LLEG
    T_LHY = T_PEL*[rotZ_rad(q1) [0 l1 0]'; [0 0 0 1]];
    T_LHR = T_LHY*[rotX_rad(q2) [0 0 0]'; [0 0 0 1]];
    T_LHP = T_LHR*[rotY_rad(q3) [0 l2 -l3]'; [0 0 0 1]];
    T_LKN = T_LHP*[rotY_rad(q4) [0 0 -l4]'; [0 0 0 1]];
    T_LAP = T_LKN*[rotY_rad(q5) [0 0 -l5]'; [0 0 0 1]];
    T_LAR = T_LAP*[rotX_rad(q6) [0 0 0]'; [0 0 0 1]];
%     T_LF  = T_LAR*[eye(3) [0 0 -l6]'; [0 0 0 1]];
    % RLEG
    T_RHY = T_PEL*[rotZ_rad(q7) [0 -l1 0]'; [0 0 0 1]];
    T_RHR = T_RHY*[rotX_rad(q8) [0 0 0]'; [0 0 0 1]];
    T_RHP = T_RHR*[rotY_rad(q9) [0 -l2 -l3]'; [0 0 0 1]];
    T_RKN = T_RHP*[rotY_rad(q10) [0 0 -l4]'; [0 0 0 1]];
    T_RAP = T_RKN*[rotY_rad(q11) [0 0 -l5]'; [0 0 0 1]];
    T_RAR = T_RAP*[rotX_rad(q12) [0 0 0]'; [0 0 0 1]];
%     T_RF  = T_RAR*[eye(3) [0 0 -l6]'; [0 0 0 1]];
    
    pCOM_PEL = T_PEL*[eye(3) c_PEL; [0 0 0 1]];     pCOM_PEL = pCOM_PEL(1:3,4);
  pCOM_TORSO = T_TORSO*[eye(3) c_TORSO; [0 0 0 1]]; pCOM_TORSO = pCOM_TORSO(1:3,4);
    pCOM_LHY = T_LHY*[eye(3) c_LHY; [0 0 0 1]];     pCOM_LHY = pCOM_LHY(1:3,4);
    pCOM_LHR = T_LHR*[eye(3) c_LHR; [0 0 0 1]];     pCOM_LHR = pCOM_LHR(1:3,4);
    pCOM_LHP = T_LHP*[eye(3) c_LHP; [0 0 0 1]];     pCOM_LHP = pCOM_LHP(1:3,4);
    pCOM_LKN = T_LKN*[eye(3) c_LKN; [0 0 0 1]];     pCOM_LKN = pCOM_LKN(1:3,4);
    pCOM_LAP = T_LAP*[eye(3) c_LAP; [0 0 0 1]];     pCOM_LAP = pCOM_LAP(1:3,4);
    pCOM_LAR = T_LAR*[eye(3) c_LAR; [0 0 0 1]];     pCOM_LAR = pCOM_LAR(1:3,4);
    pCOM_RHY = T_RHY*[eye(3) c_RHY; [0 0 0 1]];     pCOM_RHY = pCOM_RHY(1:3,4);
    pCOM_RHR = T_RHR*[eye(3) c_RHR; [0 0 0 1]];     pCOM_RHR = pCOM_RHR(1:3,4);
    pCOM_RHP = T_RHP*[eye(3) c_RHP; [0 0 0 1]];     pCOM_RHP = pCOM_RHP(1:3,4);
    pCOM_RKN = T_RKN*[eye(3) c_RKN; [0 0 0 1]];     pCOM_RKN = pCOM_RKN(1:3,4);
    pCOM_RAP = T_RAP*[eye(3) c_RAP; [0 0 0 1]];     pCOM_RAP = pCOM_RAP(1:3,4);
    pCOM_RAR = T_RAR*[eye(3) c_RAR; [0 0 0 1]];     pCOM_RAR = pCOM_RAR(1:3,4);
    
    pCOM_LLEG = (m_LHY*pCOM_LHY + m_LHR*pCOM_LHR + m_LHP*pCOM_LHP + m_LKN*pCOM_LKN + m_LAP*pCOM_LAP + m_LAR*pCOM_LAR)/m_LLEG;
    pCOM_RLEG = (m_RHY*pCOM_RHY + m_RHR*pCOM_RHR + m_RHP*pCOM_RHP + m_RKN*pCOM_RKN + m_RAP*pCOM_RAP + m_RAR*pCOM_RAR)/m_RLEG;
    pCOM = (m_PEL*pCOM_PEL + m_TORSO*pCOM_TORSO + m_LLEG*pCOM_LLEG + m_RLEG*pCOM_RLEG) / m_all;
    
    COM = [pCOM, pCOM_LLEG, pCOM_RLEG];
end