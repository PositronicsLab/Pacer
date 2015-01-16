for var in "$@"
do
      pushd .
      cd "$var"
      screen -d -m ./RUN.sh
      popd
done
