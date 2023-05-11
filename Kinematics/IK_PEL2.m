% Input: Kinematic information, x_target / Output: q_target
% x_target = [pPEL; qPEL; pLF; qLF; pRF; qRF];
function q_target = IK_PEL2(x_target)
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

%% IK Pel : Right leg

g_HR = pPEL + quat2mat(qPEL)*[0 -l1 0]';

mRF = quat2mat(qRF);
% imRF = inv(mRF);

g_F2HR = g_HR - pRF; % put foot position to zero
% f_F2HR = imRF*g_F2HR;  % Ref frame changed to foot frame
f_F2HR = mRF\g_F2HR;  % Ref frame changed to foot frame

f_F2AR = -[0 0 -l6]';
f_AR2HR = f_F2HR - f_F2AR;

t = -(f_AR2HR(3) - sqrt(f_AR2HR(2)^2 + f_AR2HR(3)^2 - l2^2))/(f_AR2HR(2) + l2);

q12 = 2*atan(t); % deg
% q12 = 2*atan(t)*180/pi; % deg

% solve q2 and q1 (Analytical Approach)
% Homogenuous Transformation matrix
g_PEL2F = pRF - pPEL;
% imPEL = inv(quat2mat(qPEL));
% rot0toF = imPEL*mRF;
% pel_PEL2F = imPEL*g_PEL2F;
rot0toF = quat2mat(qPEL)\mRF;
pel_PEL2F = quat2mat(qPEL)\g_PEL2F;

T0toF = [rot0toF pel_PEL2F ; [0 0 0 1]];

% rot6toF = eye(3);
% T6toF = [rot6toF [0 0 -l6]';[0 0 0 1]];

rot5to6 = rotX_rad(q12);
% T5to6 = [rot5to6 [0 0 0]';[0 0 0 1]];

rot0to5 = rot0toF*rot5to6';
T0to5 = T0toF*[eye(3) -[0 0  -l6]';[0 0 0 1]]*[rot5to6' -rot5to6'*[0 0 0]';[0 0 0 1]]; %0TF*6TF'*5T6' = 0T5

q8 = asin(rot0to5(3,2)); %deg %rot0to5(3,2) = sin(q2)
% q8 = asin(rot0to5(3,2))*180/pi;

SINq7 = -rot0to5(1,2)/cos(q8); %rot0to5(1,2) = -cos(q2)*sin(q1);
COSq7 = rot0to5(2,2)/cos(q8); %rot0to5(2,2) = cos(q1)*cos(q2);

q7 = atan2(SINq7,COSq7);
% q7 = atan2(SINq7,COSq7)*180/pi;

% solve q3+q4+q5 
rot0to1 = rotZ_rad(q7);
rot1to2 = rotX_rad(q8);
rot2to5 = rot1to2'*rot0to1'*rot0to5;

%rot2to5 = [  cos(q3 + q4 + q5), 0, sin(q3 + q4 + q5)]
%          [                  0, 1,                 0]
%          [ -sin(q3 + q4 + q5), 0, cos(q3 + q4 + q5)]

COS_q345 = rot2to5(1,1);    
SIN_q345 = rot2to5(1,3);

q345 = atan2(SIN_q345, COS_q345);

% solve q4
pel_HR2HP = rot0to1*rot1to2*[0 -l2 -l3]';

f_HR2HP = rot0toF'*pel_HR2HP;

f_AR2HP = f_AR2HR + f_HR2HP;

l_AR2HP = norm(f_AR2HP);

if l_AR2HP > (l4 + l5 - 0.01)
    l_AR2HP = l4 + l5 - 0.01;
end

COS_PI_minus_q4 = (l4*l4 + l5*l5 - l_AR2HP*l_AR2HP)/(2*l4*l5);
if COS_PI_minus_q4 > 1
    COS_PI_minus_q4 = 1;
elseif COS_PI_minus_q4 < -1
    COS_PI_minus_q4 = -1;
end

q10 = (pi - acos(COS_PI_minus_q4));
% q10 = (pi - acos(COS_PI_minus_q4))*180/pi;


% solve q3
T2to5 = [rot1to2' -rot1to2'*[0 0 0]';[0 0 0 1]]*[rot0to1' -rot0to1'*[0 -l1 0]';[0 0 0 1]]*T0to5; %T2to5 = T1to2'*T0to1'*T0to5;

% T2to5(1,4) = - l5*(cos(q3)*sin(q4) + cos(q4)*sin(q3)) - l4*sin(q3)
% T2to5(3,4) = - l3 - l5*(cos(q3)*cos(q4) - sin(q3)*sin(q4)) - l4*cos(q3)

A1 = -l5*sin(q10);
A2 = -l5*cos(q10) - l4;
A3 = T2to5(1,4);

B1 = -l5*cos(q10) - l4;
B2 = l5*sin(q10);
B3 = l3 + T2to5(3,4);
% 
SIN_q9 = (A3*B1 - B3*A1)/(A2*B1 - A1*B2);
COS_q9 = (A3*B2 - B3*A2)/(A1*B2 - A2*B1);

q9 = atan2(SIN_q9,COS_q9);
% q9 = atan2(SIN_q9,COS_q9)*R2D;


% solve q5
q11 = q345 - q10 - q9;
% q11 = q11*D2R; 
% q11 = q11*R2D;

%% IK Pel : Left leg

g_HR = pPEL + quat2mat(qPEL)*[0 l1 0]';

mLF = quat2mat(qLF);
% imRF = inv(mRF);

g_F2HR = g_HR - pLF; % put foot position to zero
% f_F2HR = imRF*g_F2HR;  % Ref frame changed to foot frame
f_F2HR = mLF\g_F2HR;  % Ref frame changed to foot frame

f_F2AR = -[0 0 -l6]';
f_AR2HR = f_F2HR - f_F2AR;

t = -(f_AR2HR(3) - sqrt(f_AR2HR(2)^2 + f_AR2HR(3)^2 - l2^2))/(f_AR2HR(2) - l2);

q6 = 2*atan(t); % deg
% q12 = 2*atan(t)*180/pi; % deg

% solve q2 and q1 (Analytical Approach)
% Homogenuous Transformation matrix
g_PEL2F = pLF - pPEL;
% imPEL = inv(quat2mat(qPEL));
% rot0toF = imPEL*mRF;
% pel_PEL2F = imPEL*g_PEL2F;
rot0toF = quat2mat(qPEL)\mLF;
pel_PEL2F = quat2mat(qPEL)\g_PEL2F;

T0toF = [rot0toF pel_PEL2F ; [0 0 0 1]];

% rot6toF = eye(3);
% T6toF = [rot6toF [0 0 -l6]';[0 0 0 1]];

rot5to6 = rotX_rad(q6);
% T5to6 = [rot5to6 [0 0 0]';[0 0 0 1]];

rot0to5 = rot0toF*rot5to6';
T0to5 = T0toF*[eye(3) -[0 0  -l6]';[0 0 0 1]]*[rot5to6' -rot5to6'*[0 0 0]';[0 0 0 1]]; %0TF*6TF'*5T6' = 0T5

q2 = asin(rot0to5(3,2)); %deg %rot0to5(3,2) = sin(q2)
% q8 = asin(rot0to5(3,2))*180/pi;

SINq1 = -rot0to5(1,2)/cos(q2); %rot0to5(1,2) = -cos(q2)*sin(q1);
COSq1 = rot0to5(2,2)/cos(q2); %rot0to5(2,2) = cos(q1)*cos(q2);

q1 = atan2(SINq1,COSq1);
% q7 = atan2(SINq7,COSq7)*180/pi;

% solve q3+q4+q5 
rot0to1 = rotZ_rad(q1);
rot1to2 = rotX_rad(q2);
rot2to5 = rot1to2'*rot0to1'*rot0to5;

%rot2to5 = [  cos(q3 + q4 + q5), 0, sin(q3 + q4 + q5)]
%          [                  0, 1,                 0]
%          [ -sin(q3 + q4 + q5), 0, cos(q3 + q4 + q5)]

COS_q345 = rot2to5(1,1);    
SIN_q345 = rot2to5(1,3);

q345 = atan2(SIN_q345, COS_q345);

% solve q4
pel_HR2HP = rot0to1*rot1to2*[0 l2 -l3]';

f_HR2HP = rot0toF'*pel_HR2HP;

f_AR2HP = f_AR2HR + f_HR2HP;

l_AR2HP = norm(f_AR2HP);

if l_AR2HP > (l4 + l5 - 0.01)
    l_AR2HP = l4 + l5 - 0.01;
end

COS_PI_minus_q4 = (l4*l4 + l5*l5 - l_AR2HP*l_AR2HP)/(2*l4*l5);
if COS_PI_minus_q4 > 1
    COS_PI_minus_q4 = 1;
elseif COS_PI_minus_q4 < -1
    COS_PI_minus_q4 = -1;
end

q4 = (pi - acos(COS_PI_minus_q4));
% q10 = (pi - acos(COS_PI_minus_q4))*180/pi;


% solve q3
T2to5 = [rot1to2' -rot1to2'*[0 0 0]';[0 0 0 1]]*[rot0to1' -rot0to1'*[0 -l1 0]';[0 0 0 1]]*T0to5; %T2to5 = T1to2'*T0to1'*T0to5;

% T2to5(1,4) = - l5*(cos(q3)*sin(q4) + cos(q4)*sin(q3)) - l4*sin(q3)
% T2to5(3,4) = - l3 - l5*(cos(q3)*cos(q4) - sin(q3)*sin(q4)) - l4*cos(q3)

A1 = -l5*sin(q4);
A2 = -l5*cos(q4) - l4;
A3 = T2to5(1,4);

B1 = -l5*cos(q4) - l4;
B2 = l5*sin(q4);
B3 = l3 + T2to5(3,4);
% 
SIN_q3 = (A3*B1 - B3*A1)/(A2*B1 - A1*B2);
COS_q3 = (A3*B2 - B3*A2)/(A1*B2 - A2*B1);

q3 = atan2(SIN_q3,COS_q3);
% q9 = atan2(SIN_q9,COS_q9)*R2D;


% solve q5
q5 = q345 - q4 - q3;
% q11 = q11*D2R; 
% q11 = q11*R2D;

q_target = [q1; q2; q3; q4; q5; q6; q7; q8; q9; q10; q11; q12];

end