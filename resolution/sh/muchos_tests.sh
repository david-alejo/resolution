#! /bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file

CONTADOR=$1

if [ $# -ne 5 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <path to the repository module> <number of test with each UAV number> <input_file>"
  exit 1
fi


until [ $CONTADOR -gt $2 ]; do
  
  if [ ! -d "${CONTADOR}_uavs" ]; then
    mkdir ${CONTADOR}_uavs
  fi
  cd ${CONTADOR}_uavs
  $3/build/random_test_generator $5 swarm_ --repetitions $4
  cd ..
  let CONTADOR+=1

done