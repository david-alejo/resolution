#!/bin/bash

if [ $# -ne 2 ]; then
  echo "This script changes the name of the files in the directory"
  echo "Usage: $0 <old_base_name> <new_base_name> "
  exit 1
fi

for filename in $1*
do
  new_file=${filename#$1}
  new_file=$2$new_file
#   echo $new_file
  mv $filename $new_file
done;