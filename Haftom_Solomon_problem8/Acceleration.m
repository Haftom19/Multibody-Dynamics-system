function d2q=Acceleration(dq,q,t)
% d2q=Acceleration(dq,q,t)
%   This procedure solves the acceleration problem.
%   The position and velocity problems have to be solved earlier.
% In:
%   dq - the vector of time derivatives of absolute coordinates,
%   q - the vector of absolute coordinates,
%   t - the current time instant.
% Out:
%   d2q - the vector of the second time derivatives of absolute coordinates.

Data; % Reads the mechanism dimensions from a file
q= [ 0.70 -0.20 0, 0 0.20 0, 0.20 0.30 0, 1.55 -0.35 0, 0.90 0.20 0, 0.20 -0.35 0, 0.60 -0.25 0, 0.15 -0.45 0, 0.25 0.05 0, 0.70 0 0]';

Om=[0 -1;1 0];  % Auxiliary matrix

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

% More user-friendly names
dfi1=dq(3);   dfi2=dq(6);   dfi3=dq(9);
dfi4=dq(12);   dfi5=dq(15);   dfi6=dq(18);
dfi7=dq(21);   dfi8=dq(24);   dfi9=dq(27);
dfi10=dq(30);   

% More user-friendly names
dr1=dq(1:2);   dr2=dq(4:5);   dr3=dq(7:8);
dr4=dq(10:11);   dr5=dq(13:14);   dr6=dq(16:17);
dr7=dq(19:20);   dr8=dq(22:23);   dr9=dq(25:26);
dr10=dq(28:29);   

% Right hand side of the linear equations set
% Revolute joints
gam(1:2,1)=-Rot2*sB02*dfi2^2;
gam(3:4,1)=-Rot6*sB06*dfi6^2;
gam(5:6,1)=-Rot8*sB08*dfi8^2;
gam(7:8,1)=Rot1*sA14*dfi1^2-Rot4*sB14*dfi4^2;
gam(9:10,1)=Rot1*sA12*dfi1^2-Rot2*sB12*dfi2^2;
gam(11:12,1)=Rot1*sA110*dfi1^2-Rot10*sB110*dfi10^2;
gam(13:14,1)=Rot4*sA45*dfi4^2-Rot5*sB45*dfi5^2;
gam(15:16,1)=Rot3*sA35*dfi3^2-Rot5*sB35*dfi5^2;
gam(17:18,1)=Rot2*sA23*dfi2^2-Rot3*sB23*dfi3^2;
gam(19:20,1)=Rot3*sA310*dfi3^2-Rot10*sB310*dfi10^2;
gam(21:22,1)=Rot2*sA29*dfi2^2-Rot9*sB29*dfi9^2;
gam(23:24,1)=Rot7*sA710*dfi7^2-Rot10*sB710*dfi10^2;

% Translational joints
gam(25,1)=0;
gam(26,1)=(Rot7*v67)'*(2*Om*(dr7-dr6)*dfi7+(r7-r6)*dfi7^2-Rot6*sA67*(dfi7-dfi6)^2);
gam(27,1)=0;
gam(28,1)=(Rot9*v89)'*(2*Om*(dr9-dr8)*dfi9+(r9-r8)*dfi9^2-Rot8*sA89*(dfi9-dfi8)^2);

% Driving constraints
gam(29,1)=(Rot7*u67)'*(2*Om*(dr7-dr6)*dfi7+(r7-r6)*dfi7^2-Rot6*sA67*(dfi7-dfi6)^2)-((100 * sin(1 * t))*10^-3);
gam(30,1)=(Rot9*u89)'*(2*Om*(dr9-dr8)*dfi9+(r9-r8)*dfi9^2-Rot8*sA89*(dfi9-dfi8)^2)+(( -50 * sin(1 * t))*10^-3);

% Coefficient matrix
Fq=Jacobian(q);

% Solution 
d2q=Fq\gam;
