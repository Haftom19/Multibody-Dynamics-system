function dq=Velocity(q,t)
% dq=Velocity(q,t)
%   This procedure solves the velocity problem.
%   The position problem has to be solved earlier.
% In:
%   q - the vector of absolute coordinates
%   t - the current time instant
% Out:
%   dq - the vector of time derivatives of absolute coordinates.
%

% Right hand side of the linear equations set
Ft=[zeros(28,1); -0.1 * cos(1 * t); -0.050 * cos(1 * t)];

% Coefficient matrix
Fq=Jacobian(q);

% Solution
dq=-Fq\Ft;
