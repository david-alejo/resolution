#! /bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file
# $6 --> Base output file
# $7 --> Simulator file with ModelQuad

CONTADOR=$1

if [ $# -ne 7 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <path to the repository module> <number of test with each UAV number> <base_input_file> <base_output_file> <QuadModel simulator file>"
  exit 1
fi



until [ $CONTADOR -gt $2 ]; do
  echo Entering \"${CONTADOR}_uavs\" directory
  cd ${CONTADOR}_uavs
  
  CONTADOR2=1
  until [ $CONTADOR2 -gt $4 ]; do
    
#     echo  Call : $3/build/edit_test $5$CONTADOR2 --output $6$CONTADOR2 --population $7 --generations $8 --cost_type ${10}
    $3/build/edit_test $5$CONTADOR2 --output $6$CONTADOR2 --to_quad_complete $7 --cost_type speed --time_exploration
    let CONTADOR2+=1
  done
#   echo Changing the cruise speed. Order: sed -i "s/cruise_speed = 20/cruise_speed = $9/g" $5*
#   sed -i "s/cruise_speed = 20/cruise_speed = $9/g" $6*
#   sed -i "s/debug = true/debug = false/g" $6*
  
  echo Exiting \"${CONTADOR}_uavs\" directory
  cd ..
  let CONTADOR+=1

done