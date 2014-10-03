for var in "$@"
do
      cd "$var"
      ~/Projects/Locomotion/Examples/Quadruped/SCRIPTS/clean_data.sh
      cd ~/Projects/Locomotion/Examples/Quadruped
done
for var in "$@"
do
      cd "$var"
      screen -d -m ~/Projects/Locomotion/Examples/Quadruped/SCRIPTS/run_test.bash
      cd ~/Projects/Locomotion/Examples/Quadruped
done
