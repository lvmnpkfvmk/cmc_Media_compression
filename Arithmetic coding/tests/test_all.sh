#!/bin/bash
sum1=0
sum2=0

for X in *
do
    if [ -f "$X" ]
    then
        echo checking $X
        echo checking $X
        filesize=$(stat -c '%s' $X)
        sum1=$((filesize + sum1))
	let "var1 = $filesize"
        printf "uncompressed size:\t%d\n" $filesize
        echo static model
        ../build/compress --method ppm --input $X --output foo
        filesize=$(stat -c '%s' foo)
        sum2=$((filesize + sum2))
	let "var2 = $filesize"
        printf "static model compressed size:\t%d\n" $filesize

	echo "print(round($var2 / $var1, 9))" | python3
        ../build/compress --method ppm --mode d --input foo --output bar
        diff -qs $X bar
        rm foo
        rm bar
    fi
done
echo "print(round($sum2 / $sum1, 9))" | python3

exit 0
