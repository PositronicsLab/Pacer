function [dX,dY] = attractor_cycle(X,Y,mu)
    dX = -Y + X*mu*(1 - (X.^2 + Y.^2));
    dY =  X + Y*mu*(1 - (X.^2 + Y.^2));
end