close all; clear all;
addpath('..')

NEAR_ZERO = 1e-5;

% warning('off')

stage1_as = [];
stage1_ip = [];
stage1_tr = [];
stage1_lemke = [];

svd_time = [];
stage2_sqp = [];
stage2_as = [];
stage2_ip = [];
stage2_lemke = [];
for ij = 1:2
    eval(['idyn_system',num2str(ij)]);
    nvars = nc*5
    nk = 4

    vb = v(nq+1:end);
    vq = v(1:nq);
    vqstar = v(1:nq) + a(1:nq);
    Z = ([E,D] - E*(F\[F,E']) )*R;
    j = [E,D]*(fext)*h + vb;
    K = [F,E']*(fext)*h  +  vq ;
    p = j + E*(F\(vqstar - K));
        
    qG1 = Z'*A*Z;
    qc1 = Z'*A*p + Z'*B*vqstar;

    % qA*z - qb >= 0
    CF = [zeros(nc,nvars)];
    for i = 1:nc
        CF(i,i) = MU(i,1);
        for k = 1:nc
            CF(i,nc+i*nc) = -1.0;
        end
    end
    qA1 = [ N'*[zeros(nq,size(Z,2)),Z]; % interpenetration
%             eye(nc),zeros(nc,nvars-nc); % COMPRESSIVE FORCE --encompassed by Coulomb Friction constraint
                        CF            ; % coulomb friction
          ];
    qb1 = [ N'*[vqstar;p] ;
%           zeros(nc,1)   ;
            zeros(nc,1)   ;
          ];
      
    qb1 = qb1 - 1e-2;


      
%     options = optimoptions(@quadprog,'Algorithm','trust-region-reflective','Display','off');
% 
%     tic()
%     z = quadprog(qG1,qc1,-qA1,-qb1,[],[],zeros(size(qc1)),[],[],options);
%     stage1_tr = [stage1_tr;toc()];
%     
%     options = optimoptions(@quadprog,'Algorithm','active-set','Display','off');
% 
%     tic()
%     z = quadprog(qG1,qc1,-qA1,-qb1,[],[],zeros(size(qc1)),[],[],options)
%     stage1_as = [stage1_as;toc()];
    
%     options = optimoptions(@quadprog,'Algorithm','interior-point-convex','Display','off');
% 
%     tic()
%     z = quadprog(qG1,qc1,-qA1,-qb1,[],[],zeros(size(qc1)),[],[],options)
%     stage1_ip = [stage1_ip;toc()];
%     
    O = zeros(size(qA1,1),size(qA1,1));
    MM = [qG1,-qA1';
          qA1,  O ];
    qq = [ qc1;-qb1];
    tic()
    zz = lemke(MM,qq);
    stage1_lemke = [stage1_lemke;toc()];
    z = zz(1:nvars)
    feas = qA1*z - qb1
    
%     uff1 = F\(-[F E']*R*z + (vqstar-k))/h + fID
1
    %% % STAGE 2 % %% QUADRATIC CONSTRAINT
%     options = optimoptions(@fmincon,'Algorithm','interior-point'...
%                                    ,'GradConstr','on'...
%                                    ,'GradObj','on'...
%                                    ,'Display','off');

%     tic()
%     [z2,fval,exitflag] = fmincon(@myfun,z,-qA1,-qb1,zeros(0,size(z,1)),[],zeros(size(z)),[],@(x) mycon(x,qG1,qc1,z),options);
%     feas = qA1*z2 - qb1
%     if exitflag >= 0
%         stage2_ip = [stage2_ip;toc()];
%     else
%         stage2_ip = [stage2_ip;nan];
%     end
% z2
%     feas = qA1*z2 - qb1
%     if min(feas) < -NEAR_ZERO
%         stage2_ip(end) = nan;
%     end
%     
%         options = optimoptions(@fmincon,'Algorithm','active-set'...
%                                    ,'GradConstr','on'...
%                                    ,'GradObj','on'...
%                                    ,'Display','off');
%     
%     tic()
% %     [z2,fval,exitflag] = fmincon(@myfun,z,-qA1,-qb1,zeros(0,size(z,1)),[],zeros(size(z)),[],@(x) mycon(x,qG1,qc1,z),options);
% %     feas = qA1*z2 - qb1
% %     if exitflag >= 0
%         stage2_as = [stage2_as;toc()];
% %     else
% %         stage2_as = [stage2_as;nan];    
% %     end
% z2
%     feas = qA1*z2 - qb1
%     if min(feas) < -NEAR_ZERO
%         stage2_as(end) = nan;
%     end
%             options = optimoptions(@fmincon,'Algorithm','SQP'...
%                                    ,'GradConstr','on'...
%                                    ,'GradObj','on'...
%                                    ,'Display','final-detailed');
% 
%     tic()
%     [z2,fval,exitflag] = fmincon(@myfun,z,-qA1,-qb1,zeros(0,size(z,1)),[],zeros(size(z)),[],@(x) mycon(x,qG1,qc1,z),options);
% %     feas = qA1*z2 - qb1
% %     if exitflag >= 0
%     stage2_sqp = [stage2_sqp;toc()];
% %     else
% %         stage2_sqp = [stage2_sqp;nan];
% %     end
%     z2
%     feas = qA1*z2 - qb1
%     if min(feas) < -NEAR_ZERO
%         stage2_sqp(end) = nan;
%     end
%     %}
    %% % STAGE 2 % %% NULLSPACE
    tic();
%     P = null(qG1)
    [u,s,v] = svd(qG1);
    ZERO_TOL = eps * size(qG1,1) * s(1);
    P = v(:,sum(s)<ZERO_TOL);
    svd_time = [svd_time; toc()];
    nvars_null = size(P,2);
    w = zeros(size(P,2));

    U = [F,E']*R;
    % Objective
    
    qG2 = P'*U'*(F'\(F\U*P));
    qc2 = (  z'*U'*(F'\((F\U)*P)) - vqstar'*(F'\((F\U)*P)) + K'*(F'\((F\U)*P))  )';
    

    CF2 = zeros(nc, nvars_null);
    cf2 = zeros(nc,1);
    for ii=1:nc
      % normal direction
      CF2(ii,:) = P(ii,:);
      cf2(ii) = -z(ii);

      % tangent directions
      for kk=nc*ii+1:nc:nc*nk+nc
        CF2(ii,:) = (CF2(ii,:) - (P(kk,:)))/MU(ii,1);
        cf2(ii)   = (cf2(ii)   + (z(kk)))/MU(ii,1);
      end
    end

    qA2 = [             
%                         qc1'*P           ;      % linear energy
                        P                ;      % Compressive Force
%            N'*[zeros(nq,nvars_null);Z*P] ;      % Interpenetration
                        CF2                     % Coulomb Friction
          ];



    qb2 = [              
%                         0         ;  % linear energy
                        -z        ;  % Compressive Force
%            -N'*[Z*z + p;vqstar]   ;  % Interpenetration
                        cf2          % Coulomb Friction
          ]; 

    qb2 = qb2 - 1e-2;
      
    O = zeros(size(qA2,1),size(qA2,1));
    MM2 =  [ qG2, -qG2, -qA2' ;
            -qG2,  qG2,  qA2' ;
             qA2, -qA2,   O  ];  
    qq2 = [qc2;-qc2;-qb2];
    tic();
    zz2 = lemke(MM2,qq2);
    w = zz2(1:nvars_null) - zz2(nvars_null+1:nvars_null*2)
%     options = optimoptions(@quadprog,'Algorithm','interior-point-convex','Display','off');
%     w = quadprog(qG2,qc2,-qA2,-qb2,[],[],zeros(size(qc2)),[],[],options);    
    stage2_lemke = [stage2_lemke;toc()];
    feas_w = qA2*w - qb2
    cf = z + P*w
    feas = qA1*cf - qb1
    1
    if min(feas) < -NEAR_ZERO
        stage2_lemke(end) = nan;
    end

end

% sum(isnan(stage1_as))
% data = [
%         nanmin(stage1_as),nanmin(stage1_ip),nanmin(stage1_tr),nanmin(stage1_lemke),nanmin(stage2_sqp),nanmin(stage2_as),nanmin(stage2_ip),nanmin(svd_time),nanmin(stage2_lemke);
%         nanmean(stage1_as),nanmean(stage1_ip),nanmean(stage1_tr),nanmean(stage1_lemke),nanmean(stage2_sqp),nanmean(stage2_as),nanmean(stage2_ip),nanmean(svd_time),nanmean(stage2_lemke);
%         nanmax(stage1_as),nanmax(stage1_ip),nanmax(stage1_tr),nanmax(stage1_lemke),nanmax(stage2_sqp),nanmax(stage2_as),nanmax(stage2_ip),nanmax(svd_time),nanmax(stage2_lemke);
%         ]
%     
%     data = [
%         sum(isnan(stage1_as)),sum(isnan(stage1_ip)),sum(isnan(stage1_tr)),sum(isnan(stage1_lemke)),sum(isnan(stage2_sqp)),sum(isnan(stage2_as)),sum(isnan(stage2_ip)),sum(isnan(svd_time)),sum(isnan(stage2_lemke));
%         nanmin(stage1_as),nanmin(stage1_ip),nanmin(stage1_tr),nanmin(stage1_lemke),nanmin(stage2_sqp),nanmin(stage2_as),nanmin(stage2_ip),nanmin(svd_time),nanmin(stage2_lemke);
%         nanmean(stage1_as),nanmean(stage1_ip),nanmean(stage1_tr),nanmean(stage1_lemke),nanmean(stage2_sqp),nanmean(stage2_as),nanmean(stage2_ip),nanmean(svd_time),nanmean(stage2_lemke);
%         nanmax(stage1_as),nanmax(stage1_ip),nanmax(stage1_tr),nanmax(stage1_lemke),nanmax(stage2_sqp),nanmax(stage2_as),nanmax(stage2_ip),nanmax(svd_time),nanmax(stage2_lemke);
%         ]

