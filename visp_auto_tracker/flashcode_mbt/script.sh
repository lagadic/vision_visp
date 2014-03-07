#!/bin/bash
./tracking -c "/home/fnovotny/playground/flashcode_mbt/data/config.cfg" -v -D ../flashcode_mbt/data/ -S 5 -r zbar > out.log
names=( "median" "mean" "max" )
for name in ${names[@]}
do
    echo $name
    i=0
    for medianline in $(cat out.log | grep $name)
    do
	medianlines[$i]=${medianline:`expr ${#name} + 1`}
	i=`expr $i + 1`
    done
    echo "start of file" > $name.txt
    echo "0 ${medianlines[0]} ${medianlines[1]} ${medianlines[2]} ${medianlines[3]} ${medianlines[4]} ${medianlines[5]} ${medianlines[6]}" >> $name.txt
done


for j in {1..20}
do
    ./tracking -c "/home/fnovotny/playground/flashcode_mbt/data/config.cfg" -v -D ../flashcode_mbt/data/ -S 5 -r zbar -R $j > out.log
    names=( "median" "mean" "max" )
    for name in ${names[@]}
    do
	echo $name
	i=0
	for medianline in $(cat out.log | grep $name)
	do
	    medianlines[$i]=${medianline:`expr ${#name}+1`}
	    i=`expr $i + 1`
	done
	echo "$j ${medianlines[0]} ${medianlines[1]} ${medianlines[2]} ${medianlines[3]} ${medianlines[4]} ${medianlines[5]} ${medianlines[6]}" >> $name.txt
    done

done

echo "#start of file" > plot.dat
for name in ${names[@]}
do
    echo $name    
    echo 'plot \"$name.txt\" using 1:2 with lines title,\"$name.txt\" using 1:3 with lines,\"$name.txt\" using 1:4 with lines,\"$name.txt\" using 1:5 with lines,\"$name.txt\" using 1:6 with lines,\"$name.txt\" using 1:7 with lines,\"$name.txt\" using 1:8 with lines' >> plot.dat
done