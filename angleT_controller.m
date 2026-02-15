function w_xyzT=angleT_controller(MPC_out,Ob_q,w_max)
    dwx=MPC_out(1);
    dwy=MPC_out(2);
    Tc=MPC_out(3);
    qw=Ob_q(1);
    qx=Ob_q(2);
    qy=Ob_q(3);
    qz=Ob_q(4);
    Kr=10;
    
    Ra=sqrt(qw^2+qz^2);
    Rz=qz/Ra;
    Rb=qw/Ra;
    Rx=Rb*qx-Rz*qy;
    Ry=Rz*qx+Rb*qy;

    C_x=Kr*(dwx-Ra*Rx);
    C_y=Kr*(dwy-Ra*Ry);
    C_max2=C_x^2+C_y^2;
    if abs(C_max2)>w_max^2
        C_K=sqrt(w_max^2/C_max2);
    else
        C_K=1;
    end
    C_x=C_x*C_K;
    C_y=C_y*C_K;

    ac_x=C_x*(1+2*Rx^2/(2*Ra^2-1));
    ac_y=C_y*(1+2*Ry^2/(2*Ra^2-1));
    
    %  C_max2=ac_x^2+ac_y^2;
    % if abs(C_max2)>w_max^2
    %     C_K=sqrt(w_max^2/C_max2);
    % else
    %     C_K=1;
    % end
    % ac_x=ac_x*C_K;
    % ac_y=ac_y*C_K;

    ax_yc=2*ac_y*Rx*Ry/(Ra^2-Rx^2+Ry^2);
    ay_xc=2*ac_x*Rx*Ry/(Ra^2+Rx^2-Ry^2);
    
    a_x=ac_x+ax_yc;
    a_y=ac_y+ay_xc;

    % C_max2=a_x^2+a_y^2;
    % if abs(C_max2)>w_max^2
    %     C_K=sqrt(w_max^2/C_max2);
    % else
    %     C_K=1;
    % end
    % a_x=a_x*C_K;
    % a_y=a_y*C_K;

    %min(max(data, -max), max);
    az_xy=(-Rx*a_y+Ry*a_x)/Ra;
    
    w_x=(Rb^2-Rz^2)*a_x+2*Rb*Rz*a_y;
    w_y=(Rb^2-Rz^2)*a_y-2*Rb*Rz*a_x;
    w_z=az_xy;
    
    w_xyzT=[w_x,w_y,w_z,Tc];

    % C_max2=w_xyzT(1)^2+w_xyzT(2)^2;
    % if abs(C_max2)>w_max^2
    %     C_K=sqrt(w_max^2/C_max2);
    % else
    %     C_K=1;
    % end
    % w_xyzT=[w_xyzT(1)*C_K,w_xyzT(2)*C_K,w_xyzT(3)*C_K,w_xyzT(4)];

end