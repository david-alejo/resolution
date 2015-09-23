#!/bin/bash

if [ $# -ne 6 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <nombre origen> <nombre destino> <inter_ways origen> <inter_ways desired>"
  exit 1
fi

CONTADOR=$1

until [ $CONTADOR -gt $2 ]; do
	cd ${CONTADOR}_uavs
	../copia_genetics.sh 200 $3 $4
	sed -i "s/intermediate_waypoints = $5;/intermediate_waypoints = $6;/g" $4*
	cd ..
	let CONTADOR+=1
done