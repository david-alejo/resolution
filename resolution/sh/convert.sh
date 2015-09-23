#!/bin/bash

if [ $# -ne 3 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <path to the repository module> "
  exit 1
fi

CONTADOR=$1

until [ $CONTADOR -gt $2 ]; do
	cd ${CONTADOR}_uavs
	sed -i "s/population = 12;/population = 100;/g" swarm_*
	sed -i "s/generations = 2;/generations = 100;/g" swarm_*
	sed -i "s/adaptative/manoeuvre/g" swarm_*
	$3/sh/copia_genetics.sh 200 swarm_ ms-swarm_  
	sed -i "s/manoeuvre_selection = false;/manoeuvre_selection = true;/g" ms-swarm_*
	$3/sh/copia_genetics.sh 200 swarm_ gen_  
	sed -i "s/algorithm = ParticleSwarm;/algorithm = Genetic;/g" gen_* 
	sed -i "s/initializer_type = Deterministic;/initializer_type = Deterministic;\n\tcustom_evolution = true;\n\tmutation_probability = 0.1;\n\tmutation_deviation = 2.5;\n\tcrossover_probability = 0.9;\n\tcrosover_type = OnePoint;/g" gen_*

	cd ..
	let CONTADOR+=1
done