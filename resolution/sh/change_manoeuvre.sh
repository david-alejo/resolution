#!/bin/bash

if [ $# -ne 4 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <nombre origen> <nombre destino>"
  exit 1
fi

CONTADOR=$1

until [ $CONTADOR -gt $2 ]; do
	cd ${CONTADOR}_uavs
	../copia_genetics.sh 200 $3 $4
	sed -i "s/max_z = 2;/max_z = 3.5;/g" $4*
	sed -i "s/min_z = 2;/max_z = 0.5;/g" $4*
	sed -i "s/waypoint_dimension = 2;/waypoint_dimension = 3;/g" $4*
	cd ..
	let CONTADOR+=1
done