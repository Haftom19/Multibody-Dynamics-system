function F=Constraints(q,t)
% F=Constraints(q,t)
%   This procedure cooperates with NewtonRaphson.
%   Left hand side of constraint equations is calculated.
% In:
%   q - the vector of absolute coordinates,
%   t - the current time instant.
% Out:
%   F - the calculated vector.
%

Data; % Reads the mechanism dimensions from a file
q= [ 0.70 -0.20 0, 0 0.20 0, 0.20 0.30 0, 1.55 -0.35 0, 0.90 0.20 0, 0.20 -0.35 0, 0.60 -0.25 0, 0.15 -0.45 0, 0.25 0.05 0, 0.70 0 0]';
t=0;

% position and orientation of CM the bodies from GRF
r1=q(1:2); fi1=q(3);   r2=q(4:5); fi2=q(6);   r3=q(7:8);   fi3=q(9);
r4=q(10:11); fi4=q(12);   r5=q(13:14); fi5=q(15);   r6=q(16:17);   fi6=q(18);
r7=q(19:20); fi7=q(21);   r8=q(22:23); fi8=q(24);   r9=q(25:26);   fi9=q(27);
r10=q(28:29); fi10=q(30);   

% Rotation matrices
Rot1=Rot(fi1);   Rot2=Rot(fi2);   Rot3=Rot(fi3);
Rot4=Rot(fi4);   Rot5=Rot(fi5);   Rot6=Rot(fi6);
Rot7=Rot(fi7);   Rot8=Rot(fi8);   Rot9=Rot(fi9);
Rot10=Rot(fi10);

% Constraint equations (left hand side)
% Revolute joints
F(1:2,1)=sA02-(r2+Rot2*sB02);               %constraint eq ground and body2
F(3:4,1)=sA06-(r6+Rot6*sB06);               %constraint eq ground and body6
F(5:6,1)=sA08-(r8+Rot8*sB08);               %constraint eq ground and body8
F(7:8,1)=r1+Rot1*sA14-(r4+Rot4*sB14);       %constraint eq body1 and body4
F(9:10,1)=r1+Rot1*sA12-(r2+Rot2*sB12);      %constraint eq body1 and body2
F(11:12,1)=r1+Rot1*sA110-(r10+Rot10*sB110); %constraint eq body1 and body10
F(13:14,1)=r4+Rot4*sA45-(r5+Rot5*sB45);     %constraint eq body4 and body5
F(15:16,1)=r3+Rot3*sA35-(r5+Rot5*sB35);     %constraint eq body3 and body5
F(17:18,1)=r2+Rot2*sA23-(r3+Rot3*sB23);     %constraint eq body2 and body3
F(19:20,1)=r3+Rot3*sA310-(r10+Rot10*sB310); %constraint eq body3 and body10
F(21:22,1)=r2+Rot2*sA29-(r9+Rot9*sB29);     %constraint eq body2 and body9
F(23:24,1)=r7+Rot7*sA710-(r10+Rot10*sB710); %constraint eq body7 and body10

% Translational joints
F(25,1)=f67-(fi6-fi7);                                 %constraint eq for the orientation body6 and body7
F(26,1) = (Rot7*v67)'*((r7-Rot7*sB67)-(r6+Rot6*sA67)); %constraint eq body6 and body7
F(27,1)=f89-(fi8-fi9);                                 %constraint eq for the orientation body6 and body7
F(28,1) = (Rot8*v89)'*((r9-Rot9*sB89)-(r8+Rot8*sA89)); %constraint eq body8 and body8

% Driving constraints
f1 = -((100 * sin(1 * t))*10^-3);                       % motion function 1 of time 
f2 = (( -50 * sin(1 * t))*10^-3);                        % motion function 2 of time 
F(29,1)=(Rot7*u67)'*(r7+Rot7*sB67-(r6+Rot6*sA67))-f1;   %driving constraint for body 6 and 7  
F(30,1)=(Rot9*u89)'*(r9+Rot9*sB89-(r8+Rot8*sA89))-f2;   %driving constraint for body 8 and 9