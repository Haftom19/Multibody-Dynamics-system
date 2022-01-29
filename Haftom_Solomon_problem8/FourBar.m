function [T,Q,DQ,D2Q]=FourBar()
% [T,Q,DQ,D2Q]=FourBar
% Kinematics: solving position, velocity and acceleration problems for a
% four-bar mechanism
%
% Out:
%   T   - an array containing time instances.
%   Q   - an array containing positions in subsequent time instances.
%   DQ  - an array containing velocities in subsequent time instances.
%   D2Q - an array containing accelerations in subsequent time instances.
%

% Initial approximation (at time = 0)
q= [ 0.70 -0.20 0, 0 0.20 0, 0.20 0.30 0, 1.55 -0.35 0, 0.90 0.20 0, 0.20 -0.35 0, 0.60 -0.25 0, 0.15 -0.45 0, 0.25 0.05 0, 0.70 0 0]';
dq=zeros(30,1);
d2q=zeros(30,1);
counter=0; 
dt=0.05; % Time step

% Kinematics at time t
for t=0:dt:20
    % Position problem. 
    % Initial approximation = previous solution + estimated changes due to velocity and acceleration, 
    q0=q+dq*dt+0.5*d2q*dt^2;
    q=NewtonRaphson(q0,t); 
    
    dq=Velocity(q,t);  % Velocity problem

    d2q=Acceleration(dq,q,t);  % Acceleration problem
    
    % Saving the results 
    counter=counter+1;
    T(1,counter)=t; 
    Q(:,counter)=q;
    DQ(:,counter)=dq;
    D2Q(:,counter)=d2q;
    
    % Detecting the singularity
    Fq=Jacobian(q);
    if(rank(Fq)~=length(q))
        error('Warning: Singularity detected')
    end
    
end

% plotting of the position for CM of part 4
% Position
figure(1)
subplot(2,1,1)
plot(T,Q(10,:),'r')
title('Position in X axis')
subplot(2,1,2)
plot(T,Q(11,:),'r')
title('Position in Y axis')

% Velocity
figure(2)
subplot(2,1,1)
plot(T,DQ(10,:),'g')
title('Velocity in X axis')
subplot(2,1,2)
plot(T,DQ(11,:),'g')
title('Velocity in Y axis')

% Acceleration
figure(3)
subplot(2,1,1)
plot(T,D2Q(10,:),'b')
title('Accleration in X axis')
subplot(2,1,2)
plot(T,D2Q(11,:),'b')
title('Accleration in Y axis')
