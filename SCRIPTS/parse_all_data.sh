for var in "$@"
do
      pushd .
      cd "$var"
      screen -d -m ~/Projects/Pacer/Examples/Quadruped/SCRIPTS/parse_data.sh 
      popd
done
