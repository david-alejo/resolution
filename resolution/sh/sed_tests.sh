#! /bin/bash
# Copies the tests if necessary and 
CONTADOR=$1

if [ $# -lt 6 ]; then
  echo "Usage: $0 <min_number_UAVs> <max_UAVs> <#tests to copy> <string_to_search> <string_to_substitute> <base input filename> [<base_output_file>]"
  exit 1
fi

until [ $CONTADOR -gt $2 ]; do
  
  cd ${CONTADOR}_uavs

    # Copy all input archives to the output ones
  if [ $# -eq 7 ]
  then
    echo Aqui
    CONTADOR_2=1
    until [ $CONTADOR_2 -gt $3 ]; do
      echo cp $6$CONTADOR_2 $7$CONTADOR_2
      cp $6$CONTADOR_2 $7$CONTADOR_2
      let CONTADOR_2+=1
    done
    #Substitute
#     echo sed -i "s/$4/$5/g" $7*
    sed -i "s/$4/$5/g" $7*
  else
    # ... and substitute the algorithm type
#     echo sed -i "s/$4/$5/g" $6*
#     echo $4
    sed -i "s/$4/$5/g" $6*
  fi
  

  cd ..
  let CONTADOR+=1



done


