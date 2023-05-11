% Input: Kinematic information, x_target / Output: q_target
% x_target = [pCOM; qPEL; pLF; qLF; pRF; qRF];
function [q_target, pPEL] = IK_COM(x_target)
    pCOM_target = x_target(1:3);
    qPEL_target = x_target(4:7);
    pLF_target  = x_target(8:10);
    qLF_target  = x_target(11:14);
    pRF_target  = x_target(15:17);
    qRF_target  = x_target(18:21);
        
%     x_target = [pCOM_target; qPEL_target; pLF_target; qLF_target; pRF_target; qRF_target];
    
    pPEL = [0; 0; 1000];
    cnt = 0;
    while 1
%         q = IK_PEL([pPEL; qPEL_target; pLF_target; qLF_target; pRF_target; qRF_target]);
        q = IK_PEL2([pPEL; qPEL_target; pLF_target; qLF_target; pRF_target; qRF_target]);
        COM = GET_COM(pPEL, qPEL_target, q);
        pCOM = COM(:,1); % COM = [pCOM, pCOM_LLEG, pCOM_RLEG]
        dpCOM = pCOM_target - pCOM;
        pPEL = pPEL + dpCOM;
        cnt = cnt + 1;
        norm(pCOM - pCOM_target);
        if norm(pCOM - pCOM_target) < 1e-06
            break
        end
    end
    pPEL = [pPEL(1); pPEL(2); 0.7];
    q_target = IK_PEL([pPEL; qPEL_target; pLF_target; qLF_target; pRF_target; qRF_target]);
end