#!/bin/bash

output_dir="exp_1/rand-fixed-sum-utilDist/log-uniform-discrete-perDist/4-core"
cores=4
shopt -s globstar
## read all generated yaml file one by one and test them in output directory and its subdirectories
csv_files=$(ls $output_dir/**/jobset*.csv)

for csv_file in $csv_files
do
(
    # echo "Testing $csv_file"
    result=$(build/nptest $csv_file -m $cores -w )
    # echo "Result $result"
    IFS=',' read -ra result_array <<< "$result"
    # remove additional spaces
    result_array[1]=$(echo "${result_array[1]}" | tr -d '[:space:]')
    # echo "SAG result: ${result_array[1]}"

    # if the result is 1, then the test succeeded
    if [ "${result_array[1]}" == "1" ]; then
        echo "Test succeeded"
    else
        echo "Test failed"
        # remove the failed yaml file
        rm $csv_file
    fi
)&
done
wait