function Rz=Dqz(Ob_q)
    qw=Ob_q(1,:);
    qx=Ob_q(2,:);
    qy=Ob_q(3,:);
    qz=Ob_q(4,:);
    
    Ra=sqrt(qw.^2+qz.^2);
    Rz=qz./Ra;
end