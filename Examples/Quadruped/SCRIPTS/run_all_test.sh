for var in "$@"
do
      pushd .
      cd "$var"
      screen -d -m ./RUN.sh
      cd ~/Projects/Pacer/Examples/Quadruped
      popd
done
