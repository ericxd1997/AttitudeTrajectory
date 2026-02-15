

%% Simulation

x_sim = zeros(max(size(simmodel.sym_x)), N_sim+1);
u_sim = zeros(max(size(simmodel.sym_u)), N_sim);

for i=2:max(size(ctrl_dyaw))
    ctrl_yaw(i)=ctrl_yaw(i-1)+ctrl_dyaw(i-1)*h;
end


ts=[0];
[qw0,qx0,qy0,qz0]=qwxyz2wxyz(-x0(6)/2,x0(5)/2,yaw2bz(ctrl_yaw(1)));
sim_x=[x0(1:4);qw0;qx0;qy0;qz0;x0(end)];
x_sim=sim_x;

for i=1:N_sim

        sim.set('x', sim_x);
        Du=[ctrl_qwx(i),ctrl_qwy(i),9.788];

        if ANGCTRL=="own"
            sim_u=angleT_controller(Du,sim_x(5:8),omega_Camax)+[0,0,ctrl_dyaw(i),0];%angle_controller(x_sim(8:10,i),[w,x,y,z]);
        elseif ANGCTRL=="euler"
            sim_u=angleT_Eulercontroller(Du,sim_x(5:8),ctrl_yaw(i),omega_Camax)+[0,0,ctrl_dyaw(i),0];
        elseif ANGCTRL=="quart"
            sim_u=angleT_Quartcontroller(Du,sim_x(5:8),omega_Camax)+[0,0,ctrl_dyaw(i),0];
        end
        
        sim.set('u', sim_u);
        sim_status = sim.solve();
        if sim_status ~= 0
            disp(['acados integrator returned error status ', num2str(sim_status)])
        end
        
        sim_x = sim.get('xn');
  
    
    % get simulated state
    x_sim(:,i+1) = sim_x;
    u_sim(:,i) = sim_u;
    ts(i+1)=i*h;
    
    if i>=1800
        i
    end
    
end

ts(N_sim+1)=N_sim*h;
sim_aN=2*x_sim(7,:).*x_sim(5,:)+2*x_sim(6,:).*x_sim(8,:);
sim_aE=-2*x_sim(6,:).*x_sim(5,:)+2*x_sim(7,:).*x_sim(8,:);
sim_aD=1-2*x_sim(6,:).^2-2*x_sim(7,:).^2;
% x_sim=x_sim(:,1:(end-1));
%% Plots
if ANGCTRL=="own"
            lc='b';
elseif ANGCTRL=="euler"
            lc='c';
elseif ANGCTRL=="quart"
            lc='g';
end
% figure(1); 
% States = {'h','u', 'v', 'w'};
% for i=1:length(States)
%     subplot(length(States), 1, i);
%     plot(ts, x_sim(i,:)); grid on;hold on;
%     plot(ts, x_opc(i,:));
%     plot(ts, yref(i, 2:max(size(ts)+1))); hold off;
%     ylabel(States{i});
%     xlabel('t [s]')
%     hold off;
% end

% figure(2); 
% States = {'qwx', 'qwy'};
% x_sim_qxy=rotorq(x_sim(5:8,:));
% for i=1:length(States)
%     subplot(length(States), 1, i);
%         plot(ts, x_sim_qxy(i,:)); grid on;hold on;
%     ylabel(States{i});
%     xlabel('t [s]')
%     hold off;
% end

% 
% figure(3); 
% States = {'T'};
% for i=1:length(States)
%     subplot(length(States), 1, i);
%         plot(ts, x_sim(i+8,:)); grid on;hold on;
%         plot(ts, x_opc(i+9,:)); hold off;
%     ylabel(States{i});
%     xlabel('t [s]')
%     hold off;
% end
% 
% figure(4); 
% States = {'dqwx', 'dqwy','dTc'};
% for i=1:length(States)
%     subplot(length(States), 1, i);
%     plot(ts, [uocp(i,:) uocp(i,end)]); grid on;
%     ylabel(States{i});
%     xlabel('t [s]')
%     hold off;
% end
% figure(5),plot(cost_iter)

figure(4); 
States = {'\omega_{uv}'};
for i=1:length(States)
    subplot(length(States), 1, i);
    hold on;plot(ts(2:end), sqrt(u_sim(1,:).^2+u_sim(2,:).^2),lc,"LineWidth",1);grid on;
    ylabel(States{i});
    xlabel('t [s]')
    hold off;
end

figure(5); 
States = {'qw','qx','qy','qz'};
for i=1:length(States)
    subplot(length(States), 1, i);
    hold on;plot(ts, x_sim(i+4,:));grid on;
    ylabel(States{i});
    xlabel('t [s]')
    hold off;
end

% figure(6); 
% States = {'h','u', 'v', 'w'};
% for i=1:length(States)
%     subplot(length(States), 1, i);
%     hold on;plot(ts, x_sim(i,:));grid on;
%     ylabel(States{i});
%     xlabel('t [s]')
%     hold off;
% end
figure(6); 
States = {'aN_{error}','aE_{error}'};
    subplot(length(States), 1, 1);
    hold on;plot(ts, sim_aN-Fo_aN,lc,"LineWidth",1);grid on;
    ylabel(States{1});
    xlabel('t [s]')
    subplot(length(States), 1, 2);
    hold on;plot(ts, sim_aE-Fo_aE,lc,"LineWidth",1);grid on;
    ylabel(States{2});
    xlabel('t [s]')
    hold off;

figure(7); 
States = {'aN','aE'};
    subplot(length(States), 1, 1);
    hold on;plot(ts, sim_aN,lc,"LineWidth",1);grid on;
    ylabel(States{1});
    xlabel('t [s]')
    subplot(length(States), 1, 2);
    hold on;plot(ts, sim_aE,lc,"LineWidth",1);grid on;
    ylabel(States{2});
    xlabel('t [s]')
    hold off;

figure(8); 
    hold on;plot(sim_aE,sim_aN,lc,"LineWidth",1);grid on;
    ylabel('aN');
    xlabel('aE');
    xlim([-1,1]);
    ylim([-1,1]);
    hold off;

figure(9); 
    States = {'roll[deg]','pitch[deg]','yaw[deg]'};
    eul=rad2deg(quat2eul(x_sim(5:8,:)'));
    eul(:,1)=eul(:,1)+(1-sign(eul(:,1)))*180;
    for i=1:length(States)
        subplot(length(States), 1, i);
        hold on;plot(ts, eul(:,4-i),lc,"LineWidth",1);grid on;
        ylabel(States{i});
        xlabel('t [s]')
        hold off;
    end
% 创建球体
% figure(10)
%     [X, Y, Z] = sphere(30);
%     surf(X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', 'none')
%     hold on
%     axis equal
%     plot3(sim_aN, sim_aE, sim_aD, 'r-', 'LineWidth', 2, 'DisplayName', '赤道')
%     legend
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
%     grid on