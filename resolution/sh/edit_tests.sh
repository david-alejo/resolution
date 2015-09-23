#! /bin/bash
# $1 --> The first uav number
# $2 --> The last number of uavs
# $3 --> The path to the repository (resolution)
# $4 --> Number of tests to be generated in each folder
# $5 --> Base input file
# $6 --> Base output file

CONTADOR=$1

if [ $# -lt 6 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <path to the repository module> <number of test with each UAV number> <base_input_file> <base_output_file> [<additional options>]"
  exit 1
fi



until [ $CONTADOR -gt $2 ]; do
  echo Entering \"${CONTADOR}_uavs\" directory
  cd ${CONTADOR}_uavs
  
  CONTADOR2=1
  until [ $CONTADOR2 -gt $4 ]; do
    echo Editing $5$CONTADOR2 to $6$CONTADOR2
    echo  Call : $3/build/edit_test $5$CONTADOR2 --output $6$CONTADOR2 $7 $8 $9 ${10} ${11} ${12} ${13} ${14} ${15} ${16} ${17} ${18} ${19} ${20} ${21}
    $3/build/edit_test $5$CONTADOR2 --output $6$CONTADOR2 $7 $8 $9 ${10} ${11} ${12} ${13} ${14} ${15} ${16} ${17} ${18} ${19} ${20} ${21}
    let CONTADOR2+=1
  done
  
  echo Exiting \"${CONTADOR}_uavs\" directory
  cd ..
  let CONTADOR+=1

done