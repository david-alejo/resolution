#!/bin/bash

CONTADOR=$1
CONT_2=1

# Copy all archives to the genetic ones


if [ $# -lt 5 ]; then
  echo "This script copies the files in the desired uavs directories"
  echo "Usage: $0 <min_uav> <max_uav> <number_of_tests> <input_base_name> <output_base_name> [<old_algorithm> <new_algorithm>]"
  exit 1
fi

until [ $CONTADOR -gt $2 ]; do
  cd ${CONTADOR}_uavs
  CONT_2=1
  until [ $CONT_2 -gt $3 ]; do

    cp $4$CONT_2 $5$CONT_2
    let CONT_2+=1
  done
  # ... and make a substitution if desired
  if [ $# -gt 6 ]; then
    sed -i "s/$6/$7/g" $5*
  fi
  
  cd ..
  let CONTADOR+=1
done


# sed -i "s/$4/$5/g" $3*
