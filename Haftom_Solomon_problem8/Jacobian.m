function Fq=Jacobian(q)
% Fq=Jacobian(q)
%   This procedure cooperates with NewtonRaphson.
%   The constraint Jacobian is calculated.
% In:
%   q - the vector of absolute coordinates
% Out:
%   Fq - the constraint Jacobian.
%


Data; % Reads the mechanism dimensions from a file

Om=[0 -1;1 0];  % Auxiliary matrix
%q= [ 0.70 -0.20 0, 0 0.20 0, 0.20 0.30 0, 1.55 -0.35 0, 0.90 0.20 0, 0.20 -0.35 0, 0.60 -0.25 0, 0.15 -0.45 0, 0.25 0.05 0, 0.70 0 0]';

% User-friendly names
r1=q(1:2); fi1=q(3);   r2=q(4:5); fi2=q(6);   r3=q(7:8);   fi3=q(9);
r4=q(10:11); fi4=q(12);   r5=q(13:14); fi5=q(15);   r6=q(16:17);   fi6=q(18);
r7=q(19:20); fi7=q(21);   r8=q(22:23); fi8=q(24);   r9=q(25:26);   fi9=q(27);
r10=q(28:29); fi10=q(30);   

% Rotation matrices
Rot1=Rot(fi1);   Rot2=Rot(fi2);   Rot3=Rot(fi3);
Rot4=Rot(fi4);   Rot5=Rot(fi5);   Rot6=Rot(fi6);
Rot7=Rot(fi7);   Rot8=Rot(fi8);   Rot9=Rot(fi9);
Rot10=Rot(fi10);

% Constraint Jacobian - initially contains only zeros
Fq=zeros(30,30);
% Constraint Jacobian - setting non-zero entries
% Revolute joint 0-2
Fq(1:2,4:5) = -eye(2);
Fq(1:2,6) = -Om*Rot2*sB02;

% Revolute joint 0-8
Fq(5:6,22:23) = -eye(2);
Fq(5:6,24) = -Om*Rot8*sB08;

% Revolute joint body 0-6
Fq(3:4,16:17) = -eye(2);
Fq(3:4,18) = -Om*Rot6*sB06;

% Revolute joint body 1-2
Fq(9:10,1:2) = eye(2);
Fq(9:10,3) = Om*Rot1*sA12;
Fq(9:10,4:5) = -eye(2);
Fq(9:10,6) = -Om*Rot2*sB12;

% Revolute joint body 2-3
Fq(17:18,4:5) = eye(2);
Fq(17:18,6) = Om*Rot2*sA23;
Fq(17:18,7:8) = -eye(2);
Fq(17:18,9) = -Om*Rot3*sB23;

% Revolute joint body 2-9
Fq(21:22,4:5) = eye(2);
Fq(21:22,6) = Om*Rot2*sA29;
Fq(21:22,25:26) = -eye(2);
Fq(21:22,27) = -Om*Rot9*sB29;

% Revolute joint body 3-10
Fq(19:20,7:8) = eye(2);
Fq(19:20,9) = Om*Rot3*sA310;
Fq(19:20,28:29) = -eye(2);
Fq(19:20,30) = -Om*Rot10*sB310;

% Revolute joint body 3-5
Fq(15:16,7:8) = eye(2);
Fq(15:16,9) = Om*Rot3*sA35;
Fq(15:16,13:14) = -eye(2);
Fq(15:16,15) = -Om*Rot5*sB35;

% Revolute joint body 1-10
Fq(11:12,1:2) = eye(2);
Fq(11:12,3) = Om*Rot1*sA110;
Fq(11:12,28:29) = -eye(2);
Fq(11:12,30) = -Om*Rot10*sB110;

% Revolute joint body 1-4
Fq(7:8,1:2) = eye(2);
Fq(7:8,3) = Om*Rot1*sA14;
Fq(7:8,10:11) = -eye(2);
Fq(7:8,12) = -Om*Rot4*sB14;

% Revolute joint body 4-5
Fq(13:14,10:11) = eye(2);
Fq(13:14,12) = Om*Rot4*sA45;
Fq(13:14,13:14) = -eye(2);
Fq(13:14,15) = -Om*Rot5*sB45;

% Revolute joint body 7-10
Fq(23:24,19:20) = eye(2);
Fq(23:24,21) = Om*Rot7*sA710;
Fq(23:24,28:29) = -eye(2);
Fq(23:24,30) = -Om*Rot10*sB710;

% Translational joint body 6-7
Fq(25,18) = 1;
Fq(25,21) = -1;

Fq(26,16:17) = (-Rot7*v67)';
Fq(26,18) = -(Rot7*v67)'*Om*Rot6*sA67;
Fq(26,19:20) = (Rot7*v67)';
Fq(26,21) = -(Rot7*v67)'*Om*(r7-(r6+Rot6*sA67));

% Translational joint body 8-9
Fq(27,24) = 1;
Fq(27,27) = -1;

Fq(28,22:23) = (-Rot9*v89)';
Fq(28,24) = -(Rot9*v89)'*Om*Rot8*sA89;
Fq(28,25:26) = (Rot9*v89)';
Fq(28,27) = -(Rot9*v89)'*Om*(r9-(r8+Rot8*sA89));

% Driving constraint 1
Fq(29,16:17) = -(Rot7*u67)';
Fq(29,18) = -(Rot7*u67)'*Om*Rot6*sA67;
Fq(29,19:20) = (Rot7*u67)';
Fq(29,21) = -(Rot7*u67)'*Om*(r7 - (r6+Rot6*sA67));

% Driving constraint 2
Fq(30,22:23) = -(Rot9*u89)';
Fq(30,24) = -(Rot9*u89)'*Om*Rot8*sA89;
Fq(30,25:26)= (Rot9*u89)';
Fq(30,27) = -(Rot9*u89)'*Om*(r9 - (r8+Rot8*sA89));


