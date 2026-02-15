function w_xyzT=angleT_Eulercontroller(MPC_out,Ob_q,yaw_rad,w_max)
    dwx=MPC_out(1);
    dwy=MPC_out(2);
    Tc=MPC_out(3);
    qw=Ob_q(1);
    qx=Ob_q(2);
    qy=Ob_q(3);
    qz=Ob_q(4);
    Kr=5;
    
    eul_r = quat2eul([qw;qx;qy;qz]');

    % Ra=sqrt(qw^2+qz^2);
    % Rz=qz/Ra;
    % Rb=qw/Ra;
    Rb=cos(yaw_rad/2);
    Rz=sin(yaw_rad/2);
    [w,x,y]=qwxy2wxy(dwx,dwy);

    wxyz=[w*Rb;...
            Rb*x+Rz*y;...
            Rb*y-Rz*x;...
            w*Rz];
    eul_d = quat2eul(wxyz');

    eul_d=eul_d([3,2,1]);
    eul_r=eul_r([3,2,1]);
    eul_e=eul_d-eul_r;
    for i=1:3
        if abs(eul_e(i))>pi && eul_e(i)>0
            eul_e(i)=eul_e(i)-2*pi;
        elseif abs(eul_e(i))>pi && eul_e(i)<0
            eul_e(i)=eul_e(i)+2*pi;
        end
    end

    % C_max2=eul_e(1)^2+eul_e(2)^2;
    % if abs(C_max2)>w_max^2
    %     C_K=sqrt(w_max^2/C_max2);
    % else
    %     C_K=1;
    % end
    % eul_e=eul_e.*C_K;

    eul_c(1) = eul_e(1) - sin(eul_r(2)) * eul_e(3);
	eul_c(2) = cos(eul_r(1))  * eul_e(2) + sin(eul_r(1)) * cos(eul_r(2)) * eul_e(3);
	eul_c(3) = -sin(eul_r(1)) * eul_e(2) + cos(eul_r(2)) * cos(eul_r(1)) * eul_e(3);
    
    w_xyzT=[Kr*eul_c(1),Kr*eul_c(2),Kr*eul_c(3),Tc];

    C_max2=w_xyzT(1)^2+w_xyzT(2)^2;
    if abs(C_max2)>w_max^2
        C_K=sqrt(w_max^2/C_max2);
    else
        C_K=1;
    end
    w_xyzT=[w_xyzT(1)*C_K,w_xyzT(2)*C_K,w_xyzT(3)*C_K,w_xyzT(4)];

end