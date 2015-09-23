#!/bin/bash

CONTADOR=1

until [ $CONTADOR -gt $2 ]; do

	rm $1$CONTADOR
	let CONTADOR+=1
done
