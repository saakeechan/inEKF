function  J = jacobian_mini_cheetah_leg(q,i_leg)
%jacobian 이 함수의 요약 설명 위치
%   자세한 설명 위치


%%FR , FL, RR, RL

J = zeros(3,3);

q1 = q(1+3*(i_leg-1));
q2 = q(2+3*(i_leg-1));
q3 = q(3+3*(i_leg-1));


abadLinkLength = 0.062;
hipLinkLength = 0.209;
% kneeLinkLength = 0.195;
kneeLinkLength = 0.18;
kneeLinkY_offset = 0.004;

% % bodyLength = 0.19 * 2;
% bodyLength = 0.2 * 2;
% bodyWidth = 0.049 * 2;

sideSigns = [-1;1;-1;1];



l1 = abadLinkLength;
l2 = hipLinkLength;
l3 = kneeLinkLength;
l4 = kneeLinkY_offset;
sideSign = sideSigns(i_leg);

s1 = sin(q1);
s2 = sin(q2);
s3 = sin(q3);


c1 = cos(q1);
c2 = cos(q2);
c3 = cos(q3);


c23 = c2 * c3 - s2 * s3;
s23 = s2 * c3 + c2 * s3;




J(1, 1) = 0;
J(1, 2) = l3 * c23 + l2 * c2;
J(1, 3) = l3 * c23;
J(2, 1) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
J(2, 2) = -l3 * s1 * s23 - l2 * s1 * s2;
J(2, 3) = -l3 * s1 * s23;
J(3, 1) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
J(3, 2) = l3 * c1 * s23 + l2 * c1 * s2;
J(3, 3) = l3 * c1 * s23;



end