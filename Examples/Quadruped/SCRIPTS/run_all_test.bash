for var in "$@"
do
      cd "$var"
      screen -d -m ./RUN.sh
      cd ~/Projects/Locomotion/Examples/Quadruped
done
