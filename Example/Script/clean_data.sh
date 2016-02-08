rm $1/*.mat
rm $1/*.log
find $1 -maxdepth 1 -name "driver.out*.osg" -type f -print | xargs rm
find $1 -maxdepth 1 -name "lemke.Mq*" -type f -print | xargs rm
#for i in $1data/idyn_system*; do rm $i; done
#for i in $1data/idyn_soln*; do rm $i; done
#for i in $1data/moby_cf*; do rm $i; done
