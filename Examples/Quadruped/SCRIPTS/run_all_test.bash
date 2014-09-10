for var in "$@"
do
      cd "$var"
      ~/Projects/Control-Moby/clean_data.sh 
      cd ~/Projects/Control-Moby/Examples/Quadruped
done
for var in "$@"
do
      cd "$var"
      screen -d -m ~/Projects/Control-Moby/Examples/Quadruped/SCRIPTS/run_test.bash 
      cd ~/Projects/Control-Moby/Examples/Quadruped
done
