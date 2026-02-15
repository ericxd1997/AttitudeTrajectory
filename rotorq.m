function ax_ay=rotorq(Ob_q)
    qw=Ob_q(1,:);
    qx=Ob_q(2,:);
    qy=Ob_q(3,:);
    qz=Ob_q(4,:);
    
    Ra=sqrt(qw.^2+qz.^2);
    Rz=qz./Ra;
    Rb=qw./Ra;
    Rx=Rb.*qx-Rz.*qy;
    Ry=Rz.*qx+Rb.*qy;
    
    ax_ay=[Ra.*Rx;Ra.*Ry];
end