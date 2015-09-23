#!/bin/bash

CONTADOR=$1

until [ $CONTADOR -gt $2 ]; do
	cd ${CONTADOR}_uavs
# 	sed -i "s/adaptative/manoeuvre/g" swarm_*
# 	../copia_genetics.sh 200 swarm_ pso-swarm_  
# 	sed -i "s/manoeuvre_selection = true;/manoeuvre_selection = false;/g" swarm_*
	../copia_genetics.sh 200 swarm_ gen_  
	sed -i "s/controller_type = ControllerSimple;/controller_type = ControllerQuadCatec;/g" gen_*
	sed -i "s/initializer_type = Deterministic;/initializer_type = Deterministic;\ncustom_evolution = true;\nmutation_probability = 0.1;\nmutation_deviation = 2.5;\ncrossover_probability = 0.9;\ncrosover_type = OnePoint;/g" gen_*
# 	sed -i "s/manoeuvre;/manoeuvre;\n\tmanoeuvre_selection = true;/g" pso-swarm_*

	cd ..
	let CONTADOR+=1
done