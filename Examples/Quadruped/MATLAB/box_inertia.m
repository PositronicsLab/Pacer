function J = box_inertia(x,y,z,m)

J = diag([m*(y*y + z*z)/12, ...
         m*(x*x + z*z)/12, ...
         m*(y*y + x*x)/12]);

end