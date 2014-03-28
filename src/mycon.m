    function [c,ceq,G,Geq] = mycon(x,M,q,z)
        ceq = [];
        Geq = [];
        G = (M*x + q);
        c = x'*(M*x + q) - z'*(M*z + q)+ 1e-2;
    end