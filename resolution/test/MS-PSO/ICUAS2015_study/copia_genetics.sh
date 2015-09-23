#!/bin/bash

CONTADOR=1

# Copy all archives to the genetic ones

until [ $CONTADOR -gt $1 ]; do

	cp $2$CONTADOR $3$CONTADOR
	let CONTADOR+=1
done

# ... and substitute the algorithm type
# sed -i "s/$4/$5/g" $3*
# sed -i "s/$4/$5/g" $3*
