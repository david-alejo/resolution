#! /bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder

CONTADOR=$1

if [ $# -ne 6 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <path to the repository> <number of test with each UAV number> <input_file> <output_base_filename>"
  exit 1
fi


until [ $CONTADOR -gt $2 ]; do
  
  if [ ! -d "${CONTADOR}_uavs" ]; then
    mkdir ${CONTADOR}_uavs
  fi
  cd ${CONTADOR}_uavs
  echo Calling the generator: $3/build/random_test_generator $5 $6 --repetitions $4
  $3/build/random_test_generator $5 $6 --repetitions $4
  cd ..
  let CONTADOR+=1

done