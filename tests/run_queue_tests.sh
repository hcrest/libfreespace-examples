#!/bin/sh

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

filenum=0
numloops=1

testParams[0]=" --enable-user" 
testParams[1]=" --enable-user --enable-body"
testParams[2]=" --enable-user --enable-body --enable-mouse"
testParams[3]=" --enable-user --enable-mouse"
testParams[4]=" --enable-body --enable-mouse"
testParams[5]=" --enable-body"
testParams[6]=" --enable-mouse"

rm -rf testResults
mkdir testResults

#how many times to do this
for (( i = 0; i < numloops; i++))
do
    #1 - 500ms sleep delay
    for (( j = 1; j <= 510; j += 8))
    do
	#send ms delay
	#specifying this automatically enables control
	for (( k = 5; k <= 50; k += 5))
	do
	    #do one set with product ID, the other without
	    for ((l = 0; l < 7; l++))
	    do
		./queue_test $j --control-delay=$k ${testParams[$l]} > testResults/${filenum}.txt
		filenum=`expr $filenum + 1`
	    done

	    #do one set with product ID, the other without
	    for ((l = 0; l < 7; l++))
	    do
		./queue_test $j --control-delay=$k --enable-productId ${testParams[$l]} > testResults/${filenum}.txt
		filenum=`expr $filenum + 1`
	    done    

	done

	#now do a set w/o control messages
	for (( l = 0; l < 7; l++))
	do
	    ./queue_test $j ${testParams[$l]} > testResults/${filenum}.txt
	    filenum=`expr $filenum + 1`
	done

    done



done
