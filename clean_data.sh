rm $1driver.out*.osg
rm $1pivot.steps
for i in $1lemke.Mq.*; do rm $i; done
for i in $1driver.out*.osg; do rm $i; done
for i in $1data/idyn_system*; do rm $i; done
for i in $1data/idyn_soln*; do rm $i; done
for i in $1data/moby_cf*; do rm $i; done
#find 0.1 -name 'lemke.Mq.*' | xargs 'rm'
