#!/bin/bash

CONTADOR=$1

if [ $# -ne 5 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <#tests to copy> <base input filename> <base_output_file>"
  exit 1
fi

until [ $CONTADOR -gt $2 ]; do
  
  cd ${CONTADOR}_uavs

    # Copy all archives to the genetic ones
  CONTADOR_2=1
  until [ $CONTADOR_2 -gt $3 ]; do
    cp $4$CONTADOR_2 $5$CONTADOR_2
    let CONTADOR_2+=1
  done
  # ... and substitute the algorithm type
  sed -i "s/geometry = 1 1 1/geometry = 1.2 1.2 1.0 0 0 0 0.6 0.6 0.3 0 0.3 -0.65/g" $5*
  sed -i "s/population = 100/population = 50/g" $5*
  sed -i "s/collision_detector_type = SiCoDe;/collision_detector_type = SiCoDeExtended;/g" $5*

  cd ..
  let CONTADOR+=1



done


