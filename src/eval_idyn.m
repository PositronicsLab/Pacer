close all; clear all;
addpath('..')

NEAR_ZERO = 1e-5;

% warning('off')

N_SYSTEMS = 4997;
rel_err = [];
for iteration = 1:N_SYSTEMS
  try  
  eval(['idyn_system',num2str(iteration)]);
  eval(['idyn_soln',num2str(iteration)]);
  eval(['moby_cf',num2str(iteration)]);
  catch
      continue;
  end
  
    vb = v(nq+1:end);
    vq = v(1:nq);
    vqstar = v(1:nq) + a(1:nq);
    Z = ([E,D] - E*(F\[F,E']) )*R;
    j = [E,D]*(fext)*h + vb;
    K = [F,E']*(fext)*h  +  vq ;
    p = j + E*(F\(vqstar - K));
    
%   Z*z + p  == v +M\(fext*h + R*z) 

  N(13:end,:)'*p  == N'*(v + M\fext*h)

%   vqstar == v + M\(fext + [x; zeros(6,1)])*h

  a == M\(fext + [x; zeros(6,1)])

  %qdd == inv(M)(fext + R*z + [x; zeros(6,1)])*h)

  %{
  N'*(M\N)
  N'*(M\N)
  N'*(M\N)
  S'*(M\S)
  S'*(M\T)
  T'*(M\T)
  %}

  this_rel_err = zeros(4,1);
  for c = 0:nc-1

    eval(['z1 = cfs_',num2str(c),';']);
    eval(['p1 = pts_',num2str(c),';']);
    eval(['z2 = cfs_idyn_',num2str(c),';']);
    eval(['p2 = pts_idyn_',num2str(c),';']);
    
    if(size(z1,1)>1)
        z1 = sum(z1);
        p1 = sum(p1);
    end
    z1 = z1';
    p1 = p1';
    
    p1
    p2
    
    if(norm(p1-p2) < 1e-2)
        this_rel_err(c+1) = abs((z2(3)-z1(3))./z1(3))
    else
        this_rel_err(c+1) = 0/0
    end
  end
   rel_err = [rel_err this_rel_err];
end
plot(real(rel_err(:,2:end)'));
