close all; clear all;
addpath('..')

NEAR_ZERO = 1e-5;

% warning('off')

N_SYSTEMS = 1000;
relative_error = [];
absolute_error = [];
addpath data;
for iteration = 0:1:N_SYSTEMS
    try
      eval(['true_data',num2str(iteration)]);
    catch
      continue;
    end  
      impulse_true
      impulse_rel_err = [];
      impulse_abs_err = [];
     try
      impulse_rel_err = (impulse_observed - impulse_true)./impulse_true
      impulse_abs_err = (impulse_observed - impulse_true)
      relative_error = [relative_error impulse_rel_err(3,:)'] ;
      absolute_error = [absolute_error impulse_abs_err(3,:)'] ;
     catch
     end
     
     try
      eval(['idyn_system',num2str(iteration)]);
      eval(['observed_data',num2str(iteration)]);
     catch
       continue;
     end      

      normal_true
      point_true

      impulse_observed
      normal_observed
      point_observed
end
subplot(2,1,1)
plot(real(relative_error(:,2:end)'));
axis([0 N_SYSTEMS -1 1])
subplot(2,1,2)
plot(absolute_error(:,2:end)');
