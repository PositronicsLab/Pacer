function [dX,dY] = cpg(x,y,mu,plot_function)
close all;
    [X,Y] = meshgrid(x,y);
    
    dX = -Y + X*mu*(1 - (X.^2 + Y.^2));
    dY =  X + Y*mu*(1 - (X.^2 + Y.^2));
    
    M = sqrt(dX.^2 + dY.^2);
    G = gradient(dX.^2 + dY.^2);
    cstring='rkmgcy'; % color string
    if(plot_function)
%         contour(X,Y,M)
%         hold on;
        quiver(X,Y,dX./M,dY./M,0.5);
        hold on;
        n = 0;
        for ic = 0.25:0.5:2
            n = n+1
            [T,p] = ode45(@(t,x) odefun(t,x,0.5),0:0.1:100,[1;0]*ic);
%             path(:,1)
%             path(:,2)
            plot(p(:,1),p(:,2),'Color',cstring(n),'LineWidth',3)
            plot(ic,0,[cstring(n),'o'])
        end
    axis([x(1) x(end) y(1) y(end)])
    axis square;
    xlabel('X')
    ylabel('Y')
    title('Attracting (Convergent) Limit Cycle')
    
    figureHandle = gcf;
    %# make all text in the figure to size 14 and bold
    set(findall(figureHandle,'type','text'),'fontSize',14,'fontWeight','bold')
    end

    
end

function dx = odefun(t,x,mu)
     dx = [-x(2);x(1)] + x*mu*(1 - norm(x));
end

