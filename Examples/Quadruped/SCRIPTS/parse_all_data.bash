for var in "$@"
do
      cd "$var"
      screen -d -m ~/Projects/Control-Moby/Examples/Quadruped/SCRIPTS/parse_data.bash 
      cd ~/Projects/Control-Moby/Examples/Quadruped
done
