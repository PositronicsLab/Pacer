for var in "$@"
do
      pushd .
      cd "$var"
      /Users/samzapo/Projects/Pacer-expermental/example/test-scripts/clean_data.sh .
      screen -d -m ./RUN.sh
      popd
done
