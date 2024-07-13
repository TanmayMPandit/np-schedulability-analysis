#!/bin/bash
for jobset_index in {0..99}; do
# Construct the jobset file name
  jobset_file="100-jitter/2.40-util/jobsets/jobset-log-uniform-discrete_${jobset_index}.csv"
  build/nptest "$jobset_file" -m 4 -f "0.74,0.8,0.87,0.94,1.0" &
done

wait