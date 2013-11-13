filenames = ["links-crb-global.dat";
             "links-crb-link.dat";
             "links-crb-linkinertial.dat";
             "links-crb-joint.dat"]
h = figure; hold on;
for i = 1:size(filenames,1);
  file = strtrim(filenames(i,:)); 
  data = dlmread(file," ");
  l2nrm = sqrt(sum((data(:,2:end).^2)'));
  %plot(l2nrm);
  plot(l2nrm,"Color",[i/4 1-i/4 1]);
end
axis([0,300,0,10])
legend(filenames)
xlabel("iterations [dt = 0.001]")
ylabel("l2-norm generalized coords")
print regression_plot.png
