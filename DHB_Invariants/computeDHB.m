%% Functions to impement the velocity-based Denavit–Hartenberg inspired Bidirectional (DHB) invariant representation
% See: D. Lee, R. Soloperto, and M. Saveriano, "Bidirectional invariant
%      representation of rigid body motions and its application to gesture
%      recognition and reproduction", Auton. Robots, 42(1):125–145, 2018.

%% Compute DHB invariants (velocity-based)
% Input: v -> linear velocity (Nx3 array) 
%        w -> angular velocity (Nx3 array)
%
% Output: m_p       -> first linear velocity invariant (N-2 array)
%         theta_p_1 -> second linear velocity invariant (N-2 array)
%         theta_p_2 -> third linear velocity invariant (N-2 array)
%
%         m_r       -> first angular velocity invariant (N-2 array)
%         theta_r_1 -> second angular velocity invariant (N-2 array)
%         theta_r_2 -> third angular velocity invariant (N-2 array)
%
%         H_p -> Initial linear frame
%         H_r -> Initial angular frame

function [m_p, theta_p_1, theta_p_2, m_r, theta_r_1, theta_r_2, H_p, H_r] = computeDHB(v, w)

[loop_iter, ~] = size(v);

% Compute frames
P_ex = c_ex(v(1,:), [1 0 0]);
P_ex2 = c_ex(v(2,:), P_ex);
P_ey = c_ey(P_ex, P_ex2, [P_ex(2)-P_ex(3) P_ex(3)-P_ex(1) P_ex(1)-P_ex(2)]/norm([P_ex(2)-P_ex(3) P_ex(3)-P_ex(1) P_ex(1)-P_ex(2)]));
P_ez = cross(P_ex,P_ey);
if norm(P_ez) > 1e-10
    P_ez = P_ez/norm(P_ez);
end

H_p = eye(4);
H_p(1:3,1:3) = ([P_ex', P_ey', P_ez' ]);
H_p(1:3,4) = v(1,:);

T_ex = c_ex(w(1,:), [1 0 0]);
T_ex2 = c_ex(w(2,:), T_ex);
T_ey = c_ey(T_ex, T_ex2, [0 1 0]);
T_ez = cross(T_ex,T_ey);
if norm(T_ez) > 1e-10
    T_ez = T_ez/norm(T_ez);
end

H_r = eye(3);
H_r(1:3,1:3) = ([T_ex', T_ey', T_ez' ]);

m_p = zeros(loop_iter-2,1);
theta_p_1 = zeros(loop_iter-2,1);
theta_p_2 = zeros(loop_iter-2,1);

m_r = zeros(loop_iter-2,1);
theta_r_1 = zeros(loop_iter-2,1);
theta_r_2 = zeros(loop_iter-2,1);

% Compute invariant values
for i = 1:loop_iter-2  
    P_ex3 = c_ex(v(i+2,:), P_ex2);
    P_ey2 = c_ey(P_ex2, P_ex3, P_ey);
    [m_p(i,1), theta_p_1(i,1), theta_p_2(i,1)] = c_values(v(i,:), P_ex, P_ex2, P_ey, P_ey2);
    
    P_ex = P_ex2;
    P_ex2 = P_ex3;
    P_ey = P_ey2;

    T_ex3 = c_ex(w(i+2,:), T_ex2);
    T_ey2 = c_ey(T_ex2, T_ex3, T_ey);
    [m_r(i,1), theta_r_1(i,1), theta_r_2(i,1)] = c_values(w(i,:), T_ex, T_ex2, T_ey, T_ey2);
    
    T_ex = T_ex2;
    T_ex2 = T_ex3;
    T_ey = T_ey2;
end

end

%% Compute the three invariant values given linear or angular frame axes (x,y)
function [m, theta1, theta2] = c_values(u, ex, ex2, ey, ey2)
    m = ex*u';
    theta1 = atan2((cross(ex,ex2)*ey'),(ex*ex2'));
    theta2 = atan2((cross(ey,ey2)*ex2'),(ey*ey2'));
end

%% Compute the x axis of linear or angular frame
function ex = c_ex(u, epx)
    ex = u;
    n_ex = norm(ex);
    if n_ex>1e-10
        ex = ex/n_ex;
    else
        ex = epx;
    end
end

%% Compute the y axis of linear or angular frame
function ey = c_ey(ex, ex2, eyp)
    ey   = cross(ex, ex2);
    n_ey = norm(ey);
    if(n_ey > 1e-10)
        ey = ey/n_ey;
    else
        ey = eyp;
    end
end
