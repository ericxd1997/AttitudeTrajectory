function w_xyzT=angleT_Quartcontroller(MPC_out,Ob_q,w_max)
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
    Rx=Rb*qx+Rz*qy;
    Ry=-Rz*qx+Rb*qy;
%%pass

    % 
    % if Rb<0
    %     Rb=-Rb;
    %     Rz=-Rz;
    % end
    % 
    % [Ma,Mx,My]=qwxy2wxy(dwx,dwy);
    % 
    % dqw=Ma*Rb;
    % dqx=Rb*Mx+My*Rz;
    % dqy=Rb*My-Mx*Rz;
    % dqz=Ma*Rz;
    % 
    % dRa=sqrt(dqw^2+dqz^2);
    % dRz=dqz/dRa;
    % dRb=dqw/dRa;
    % dRx=dRb*dqx+dRz*dqy;
    % dRy=-dRz*dqx+dRb*dqy;
    % 
    % if dRb<0
    %     dRb=-dRb;
    %     dRz=-dRz;
    % end
    % 
    % w_x=Kr*(dRx-Rx);
    % w_y=Kr*(dRy-Ry);
    % w_z=Kr*(dRz-Rz);
%%pass 结果雷同


%false%%
    [Ma,Mx,My]=qwxy2wxy(dwx,dwy);

    q0=Ma*Rb;
    q1=Rb*Mx+My*Rz;
    q2=Rb*My-Mx*Rz;
    q3=Ma*Rz;

    p0=-qw;
    p1=qx;
    p2=qy;
    p3=qz;

    pq=[p0*q0-p1*q1-p2*q2-p3*q3;...
        p0*q1+p1*q0+p2*q3-p3*q2;...
        p0*q2-p1*q3+p2*q0+p3*q1;...
        p0*q3+p1*q2-p2*q1+p3*q0];

    if pq(1)>0
        w_x=Kr*pq(2);
        w_y=Kr*pq(3);
        w_z=Kr*pq(4);
    else
        w_x=-Kr*pq(2);
        w_y=-Kr*pq(3);
        w_z=-Kr*pq(4);
    end
% false
    

     %%

    % ac_x=Kr*(dwx-Ra*Rx)*(1+2*Rx^2/(2*Ra^2-1));
    % ac_y=Kr*(dwy-Ra*Ry)*(1+2*Ry^2/(2*Ra^2-1));
    % 
    % ax_yc=2*ac_y*Rx*Ry/(Ra^2-Rx^2+Ry^2);
    % ay_xc=2*ac_x*Rx*Ry/(Ra^2+Rx^2-Ry^2);
    % 
    % a_x=ac_x+ax_yc;
    % a_y=ac_y+ay_xc;
    % az_xy=(-Rx*a_y+Ry*a_x)/Ra;
    % 
    % w_x=(Rb^2-Rz^2)*a_x+2*Rb*Rz*a_y;
    % w_y=(Rb^2-Rz^2)*a_y-2*Rb*Rz*a_x;
    % w_z=az_xy;

    w_xyzT=[w_x,w_y,w_z,Tc];

    C_max2=w_xyzT(1)^2+w_xyzT(2)^2;
    if abs(C_max2)>w_max^2
        C_K=sqrt(w_max^2/C_max2);
    else
        C_K=1;
    end
    w_xyzT=[w_xyzT(1)*C_K,w_xyzT(2)*C_K,w_xyzT(3)*C_K,w_xyzT(4)];
end