% Data.m – a file with mechanism dimensions and auxiliary quantities

sA02=[0;0]; sB02=[0;-0.20];             % Revolute joint 0-2
sA08=[0.1;-0.7]; sB08=[-0.05;-0.25];    % Revolute joint 0-8
sA06=[0;-0.4]; sB06=[-0.2;-0.05];       % Revolute joint 0-6
sA12=[-0.9;0.3]; sB12=[-0.2;-0.1];      % Revolute joint 1-2
sA23=[-0.30;0.1]; sB23=[-0.50;0];       % Revolute joint 2-3
sA29=[0.30;0.10]; sB29=[0.05;0.25] ;    % Revolute joint 2-9
sA310=[0.30;-0.10]; sB310=[-0.20;0.20]; % Revolute joint 3-10
sA35=[0.3;0.1]; sB35=[-0.4;0.2];        % Revolute joint 3-5
sA110=[0.1;0.2]; sB110=[0.1;0];         % Revolute joint 1-10
sA14=[0.80;-0.40]; sB14=[-0.05;-0.25];  % Revolute joint 1-4
sA45=[-0.25;0.35]; sB45=[0.40;-0.20];   % Revolute joint 4-5
sA710=[0.20;0.05]; sB710=[0.10;-0.20];  % Revolute joint 7-10

f67=0; v67=[-1;4]; sA67=[0.2;0.05]; sB67=[-0.2;-0.05];  % Translational joint 6-7
f89=0; v89=[-5;1]; sA89=[0.05;0.25]; sB89=[-0.05;-0.25];  % Translational joint 8-9

u67=[0.9701;0.2425]; % Unit vector of driving constraint 
u89=[0.1961;0.9805]; % Unit vector of driving constraint
