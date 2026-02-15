Kc=5;
omega_Kmax=1/(2*cos(angle_max/2)^2-1);
omega_Camax=omega_max/omega_Kmax;
A_d = exp(-Kc*h);
B_d = 1 - exp(-Kc*h);
% A_d = 1-Kc*h;
% B_d = Kc*h;
% x(k+1) = A_d*x(k) + B_d*u(k);
Fo_aN=x0(5);
Fo_aE=x0(6);
for i=2:N_sim+1
    % Fo_aN(i)=A_d*Fo_aN(i-1) + B_d*ctrl_aN(i-1);
    % Fo_aE(i)=A_d*Fo_aE(i-1) + B_d*ctrl_aE(i-1);
    C_aN=(ctrl_aN(i-1)-Fo_aN(i-1))*Kc;
    C_aE=(ctrl_aE(i-1)-Fo_aE(i-1))*Kc;
    C_max2=C_aN^2+C_aE^2;
    if abs(C_max2)>omega_Camax^2
        C_K=sqrt(omega_Camax^2/C_max2);
    else
        C_K=1;
    end

    Fo_aN(i)=Fo_aN(i-1) + C_aN*C_K*h;
    Fo_aE(i)=Fo_aE(i-1) + C_aE*C_K*h;
end


figure(6); 
States = {'aN_e','aE_e'};
    subplot(length(States), 1, 1);
    hold on;plot(ts, zeros(1,N_sim+1),'r--',"LineWidth",3);grid on;
    ylabel(States{1});
    xlabel('t [s]')
    subplot(length(States), 1, 2);
    hold on;plot(ts, zeros(1,N_sim+1),'r--',"LineWidth",3);grid on;
    ylabel(States{2});
    xlabel('t [s]')
    hold off;

figure(7); 
    subplot(2, 1, 1);
    hold on;plot(ts, Fo_aN,'r--',"LineWidth",3);
    subplot(2, 1, 2);
    hold on;plot(ts, Fo_aE,'r--',"LineWidth",3);

figure(8); 
    hold on;plot(Fo_aE,Fo_aN,'r--',"LineWidth",3);grid on;
    hold off;