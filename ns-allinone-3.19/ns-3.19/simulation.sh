#!/bin/bash

mkdir result/n$1_t$2_s$3_d$4
list="0.1 0.2 0.4 0.6 0.8 1.0 1.2 1.4 1.6"
for lambda in $list
do
   for channel in `1 1 2`
   do
      for random in `seq 1 1 3`
      do
            echo ./waf --run "scratch/ulp -n=$1 -t=$2 -c=$channel -l=$lambda -r=$random -s=$3 -d=$4"
            ./waf --run "scratch/ulp -n=$1 -t=$2 -c=$channel -l=$lambda -r=$random -s=$3 -d=$4"
      done

   done
done
