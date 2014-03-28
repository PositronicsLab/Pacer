% Main function, run "test"
function test
  NUM_EEFS = 4;

    foot = 1;
  
  figure;
  axis equal; hold on;

  dt = 0.001

  % all foot origins at [0,0,0] for now
  x0 = [0.11564, 0.0475, -0.0922774;
        0.11564, -0.0475, -0.0922774;
        -0.0832403, 0.0475, -0.0922774;
        -0.0832403, -0.0475, -0.0922774]'
   
  % plot LF foot origin
%   plot(x0(1,1),x0(3,1),'ro')

  X = []; Y = []; U = []; V = [];
%   for conv = 0.05:0.05:2
  for i = -0.08:0.005:0.08
  for j = -0.08:0.005:0.08
      x = x0;
      
      % sample from feasible region of the space
      x(:,foot) = x(:,foot) + [i;0;j];
      
      % calculate differential equation at this point
      xd = osc(x,x0);

      % plot vector field for x-z plane of LF foot
      X = [X; x(1,foot)];
      Y = [Y; x(3,foot)];    
      U = [U;xd(1,foot)];
      V = [V;xd(3,foot)];
  end
  end
  quiver(X,Y,U,V,2)
  
  % plot a few paths 
  X = []; Y = [];Z = [];
  x = x0;
  x(1,[1,4]) = x(1,[1,4]) + 0.02;
  x(1,[2,3]) = x(1,[2,3]) - 0.01;
  x(2,:) = x(2,:) - 0.01;
  x
%   plot(x(1,foot),x(3,foot),'go');

for j=1:10
  X = []; Y = []; Z = [];
  x = x0 + randn(size(x0))*0.02
  for i = 1:10000;
      xd = osc(x,x0);
      x = x + xd*dt;
      X = [X;x(1,foot)];
      Y = [Y;x(2,foot)];   
      Z = [Z;x(3,foot)];   
  end
  plot(X,Z,'r-');
end
%   plot3(X,Y,Z,'r-');


  

end

function xd = osc(x,x0)
  NUM_EEFS = 4;

    xd = zeros(size(x));

    % coupling matrix (trot)
    C = [ 0 -1 -1  1;
         -1  0  1 -1;
         -1  1  0 -1;
          1 -1 -1  0];

     % length of step (2cm)
     Ls = 0.04;
          
     % duty factor 55%
     Df = 0.75;
     
     %Forward Velocity 0.1 (10 cm/s)
     Vf = 0.025;
     
     % transition rate of Sp1 Sp2 
     % NOTE: a good value for this var is not stated in the paper
     bp = 1000;

     % height of step (2cm)
     Hs = [0.02 0.02 0.02 0.02]'*2;

     % convergence rate to the limit cycle
     % NOTE: good values for these vars are not stated in the paper
     % this value refers to 
     a = 10;
     b = -1;
     c = 10;

     bf = 1000;
     ztd = 0.1;
     
     % x,y,z bar variables
     xb = x - x0;
     %
      Cp = 0;
      for i=1:NUM_EEFS
        for j=1:NUM_EEFS
          Cp = Cp + C(i,j)*Hs(i)*xb(3,j)/Hs(j);
        end
      end

      for i=1:NUM_EEFS
        xbar = xb(:,i);
        
        % gait width var (i'm ignoring the y axis in this example)
        dyc = 0;

        % Eq 5
        Sp1 = 1.0/(exp(-bp*xbar(3)) + 1.0) ;
        
        % Eq 6
        Sp2 = 1.0/(exp( bp*xbar(3)) + 1.0);

        % Eq 4
        ws = pi * (Vf/Ls) * ((Df*Sp1)/(1.0-Df)  + Sp2) ;

        % Eq 1,2,3
        oscil = 1.0 - (4*xbar(1)*xbar(1))/(Ls*Ls) - xbar(3)*xbar(3)/(Hs(i)*Hs(i)) ;
        xd(:,i)=[
            a*(oscil)*xbar(1)+ ws*Ls*xbar(3)/(2*Hs(i));
            b*(xbar(2) + dyc);
            c*(oscil)*xbar(3)- ws*2*Hs(i)*xbar(1)/Ls + Cp;
        ];
    
        Sf1 = 1.0/(exp(-bf*(xbar(3) - ztd*Hs(i))) + 1.0);
        Sf2 = 1.0/(exp( bf*(xbar(3) - ztd*Hs(i))) + 1.0);
         
        xd(:,i)= xd(:,i)*Sf1 - [Vf,0,0]'*Sf2;
      end
end