for i in driver.out-*.osg; do\
  osgconv $i $i.pov;\
  #osgconv $i.iv $i.pov;\
  povray +I$i.pov +A +Q9;\
done
