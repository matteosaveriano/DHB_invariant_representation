% Author: Matteo Saveriano - 01.03.18

clear
close all

%% Construct 6DOF trajectory
dt = 0.1; % sample time
tf = 4;   % total time 
N = 1000; % number of samples

T = linspace(0,tf,N)';

x = 0.1*exp(T);
y = 5+1.5*sin(T);
z = cos(T);

roll  = 0.5*sin(T);
pitch = cos(T);
yaw   = 0.1*T;

%% Compute velocity (twist)
twists(:,4:6) = diff([x y z],1,1)/dt;

orientRate = diff([roll pitch yaw],1,1)/dt;
for i=1:N-1
    Tr = [ 1           0           -sin(pitch(i));...
          0   cos(roll(i))  cos(pitch(i))*sin(roll(i));...
          0  -sin(roll(i))  cos(pitch(i))*cos(roll(i)) ];

    twists(i,1:3) = (Tr * orientRate(i,:)')';
    
    a(i) = norm(twists(i,1:3));
end

                                 
%% Compute velocity based DHB invariants
[m_v, theta_v_1, theta_v_2, m_w, theta_w_1, theta_w_2, Hv0, Hw0] = computeDHB(twists(:,4:6), twists(:,1:3), 'vel');
invariants = [m_v, theta_v_1, theta_v_2, m_w, theta_w_1, theta_w_2];

%% Plot invariant trajectories
figure('NumberTitle', 'off', 'Name', 'Cartesian velocity to DHB');
dhbInvNames = {'m_v' '\theta_v^1' '\theta_v^2' 'm_{\omega}' '\theta_{\omega}^1' '\theta_{\omega}^1'};
for i=1:6
    subplot(2,3,i)
    plot(T(1:end-3),invariants(:,i),'k','LineWidth',2)
    ylabel(dhbInvNames{i});
    grid on
end

%% Reconstruct original trajectory
[vr, wr] = reconstructTrajectory(invariants, Hv0, Hw0, 'vel');

%% Compute reconstruction error    
errSP = [(vr - twists(1:N-3,4:6)).^2 (wr - twists(1:N-3,1:3)).^2];

% Compute rmse error
for i=1:6
    err(i) = sum(errSP(:,i));
end

RMSE = sqrt([sum(err(1:3)) sum(err(4:6))]./(N-3));
disp(['Reconstruction errors (RMSE): ' num2str(RMSE)])

%% Plot original and reconstructed velocity
figure('NumberTitle', 'off', 'Name', 'DHB to Cartesian velocity');
for i=1:6
    subplot(2,3,i)
    plot(T(1:end-1),twists(:,i),'g','LineWidth',4)
    hold on;
    if(i<4)
        plot(T(1:end-3),wr(:,i),'b','LineWidth',2)
        ylabel(['\omega_' num2str(i)])
    else
        plot(T(1:end-3),vr(:,i-3),'b','LineWidth',2)
        ylabel(['x_' num2str(i-3)])
    end
    grid on
end
