#!/bin/bash

if [ $# -ne 2 ]; then
  echo "This script changes the extension of all files in the directory"
  echo "Usage: $0 <old_extension> <new_extension> "
  exit 1
fi

for filename in *.$1
do
  new_file=${filename%$1}
  new_file=$new_file$2
#   echo $new_file
  mv $filename $new_file
done;