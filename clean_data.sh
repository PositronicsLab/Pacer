rm $1*.osg
rm $1pivot.steps
for i in $1lemke.Mq.*; do rm $i; done
for i in $1*.osg; do rm $i; done
for i in $1idyn_system*; do rm $i; done
for i in $1idyn_soln*; do rm $i; done
for i in $1moby_cf*; do rm $i; done
#find 0.1 -name 'lemke.Mq.*' | xargs 'rm'
