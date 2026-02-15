function [w,x,y]=qwxy2wxy(qwx,qwy)
    a=qwx^2+qwy^2;
    w=sqrt(1/2+sqrt(1-4*a)/2);
    x=qwx/w;
    y=qwy/w;
    
end