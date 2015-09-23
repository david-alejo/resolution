#! /bin/bash
#./muchos_tests.sh 2 8 ~/resolution/ 200 swarm_

# ./copia_genetics.sh $1 $2 200 swarm_ genetics_
# ./copia_tests.sh $1 $2 200 swarm_ swarmadap_
# ./copia_tests.sh $1 $2 200 genetics_ geneticsadap_

if [ $# -ne 2 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs>"
  exit 1
fi
  
  ../copia_genetics.sh $1 $2 200 swarm_ genetics_
  ../edit_tests.sh $1 $2 ~/resolution/ 200 genetics_ genet../edit_tests.sh 2 8 ~/resolution/ 200 genetics_ genetics_ --pcross 0.8 --crossover_type OnePoint --pmut 0.1 --mutator_type Gaussian
  ~/resolution/build/multitest multitest_random
  ~/resolution/build/multitest multitest_random_gen
