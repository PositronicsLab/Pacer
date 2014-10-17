for var in "$@"
do
      cd "$var"
      screen -d -m ~/Projects/Locomotion/Examples/Quadruped/SCRIPTS/parse_data.sh 
      cd ~/Projects/Locomotion/Examples/Quadruped
done
