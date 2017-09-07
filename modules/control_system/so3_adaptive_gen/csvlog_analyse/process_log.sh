#!/bin/bash
echo "$0 $#"
if [ $# -gt 1 ]
then
    DATAFOLDER=$2
else
    DATAFOLDER="./"
fi

echo "Using data folder $DATAFOLDER"
mkdir -p $DATAFOLDER

/Users/xuhao/anaconda/bin/ulog2csv $1 -o $DATAFOLDER
for f in $DATAFOLDER/*.csv
do
	echo "Processing $f"
    sed -i '' '1d' $f
done
