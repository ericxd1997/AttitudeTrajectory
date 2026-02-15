function [qw,qx,qy,qz]=qwxy2wxy(qwx,qwy,bz)
    a=qwx^2+qwy^2;
    w=sqrt(1/2+sqrt(1-4*a)/2);
    x=qwx/w;
    y=qwy/w;
    b=sqrt(1-bz^2);
    qw=w*b;
    qx=b*x+y*bz;
    qy=b*y-x*bz;
    qz=w*bz;
end