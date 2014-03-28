    function [f, G] = myfun(x)
        f = x'*x;
        G = x;
    end
