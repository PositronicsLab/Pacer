for var in "$@"
do
      pushd .
      cd "$var"
      screen -d -m ~/Projects/Locomotion/Examples/Quadruped/SCRIPTS/parse_data.sh 
      popd
done
