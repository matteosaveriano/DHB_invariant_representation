%% Functions that reconstruct a Cartesian trajectory from its DHB invariant representation
% See: D. Lee, R. Soloperto, and M. Saveriano, "Bidirectional invariant
%      representation of rigid body motions and its application to gesture
%      recognition and reproduction", Auton. Robots, 42(1):125–145, 2018.

%% Reconstruct Cartesian trajectory (position or velocity)
% Input: invariants -> DHB invariants ([N-2]x6 array) 
%        Hv0 -> Initial linear frame
%        Hw0 -> Initial angular frame
%
% Output: v -> position or linear velocity trajectory ([N-2]x3 array)
%         w -> relative rotation vector or angular velocity trajectory ([N-2]x3 array)

function [v, w] = reconstructTrajectory(invariants, Hv0, Hw0, method)
    mv   = invariants(:,1);
    tv_1 = invariants(:,2);
    tv_2 = invariants(:,3);
    mw   = invariants(:,4);
    tw_1 = invariants(:,5);
    tw_2 = invariants(:,6);

    N = size(tv_1,1);

    v = zeros(N,3);
    w = zeros(N,3);

    if(strcmp(method, 'pos'))
        coef = 1;
    else
        coef = 0;
    end

    Hv0(4,4) = coef;

    for i = 1:N
        if(strcmp(method, 'pos'))
            v(i,:) = Hv0(1:3,4)';
            Rp = rotY(tv_1(i))*rotX(tv_2(i));
            P = [mv(i) 0 0]';
            H = [Rp P; 0 0 0 coef];
            Hv0 = Hv0 * H;
        else
            Rp = rotY(tv_1(i))*rotX(tv_2(i));
            P = [mv(i) 0 0]';
            H = [Rp P; 0 0 0 coef];
            Hv0 = Hv0 * H;
            v(i,:) = Hv0(1:3,4)';
        end

        w(i,:) = (Hw0*[mw(i); 0; 0])';
        Rr = rotY(tw_1(i))*rotX(tw_2(i));
        Hw0 = Hw0 * Rr;
    end 
end

%% Elementary rotation around the x axis
function R = rotX(phi)
    R = [1        0         0; ...
         0 cos(phi) -sin(phi); ...
         0 sin(phi)  cos(phi)];
end

%% Elementary rotation around the y axis
function R = rotY(beta)
    R = [cos(beta) 0 sin(beta); ...
                 0 1         0; ...
        -sin(beta) 0 cos(beta)];
end
