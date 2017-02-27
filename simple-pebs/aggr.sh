RESET_VALUES="1500 1400 1300 1200 1100 1000 900"
BUFFER_SIZES="4194304 1048576 262144 65536"

PWD_OLD=`pwd`

for b in $BUFFER_SIZES; do
    echo "reset_value,elapsed_time,ds_overflow,pebs_events,pmc1_overflow,cache_misses" >> ${PWD_OLD}/${b}.csv

    for r in $RESET_VALUES; do  
	elapsed_time=0
	ds_overflow=0
	pmc1_overflow=0
	pebs_events=0
	cache_misses=0
	
	for i in 1 2 3; do
	    cd ${b}/${i}

	    #elapsed_time_i=`grep "Elapsed time" ${r}.log | sed -e "s/Elapsed time://"`
	    elapsed_time_i=`grep "Time in seconds" ${r}.log | sed -e "s/Time in seconds =//"`
	    elapsed_time=`echo $elapsed_time + $elapsed_time_i | bc -l`

	    ds_overflow_i=`grep "# of DS overflow" ${r}.dmesg | sed -e "s/.*# of DS overflow inturrupt: \([0-9]*\)/\1/"`
	    ds_overflow=`echo $ds_overflow + $ds_overflow_i | bc` 

	    pebs_events_i=`grep "# of DS overflow" -A 1 ${r}.dmesg | tail -n 1 | sed -e "s/.*# of events: \([0-9]*\)/\1/"`
	    pebs_events=`echo $pebs_events + $pebs_events_i | bc`

	    pmc1_overflow_i=`grep "# of PMC1 overflow" ${r}.dmesg | sed -e "s/.*# of PMC1 overflow inturrupt: \([0-9]*\)/\1/"`
	    pmc1_overflow=`echo $pmc1_overflow + $pmc1_overflow_i | bc` 

	    cache_misses_i=`grep "# of PMC1 overflow" -A 1 ${r}.dmesg | tail -n 1 | sed -e "s/.*# of events: \([0-9]*\)/\1/"`
	    cache_misses=`echo $cache_misses + $cache_misses_i | bc`

	    cd $PWD_OLD
	done

	elapsed_time=`echo $elapsed_time / 3 | bc -l`
	ds_overflow=`echo $ds_overflow / 3 | bc`
	pebs_events=`echo $pebs_events / 3 | bc`
	pmc1_overflow=`echo $pmc1_overflow / 3 | bc`
	cache_misses=`echo $cache_misses / 3 | bc`

	echo "$r,$elapsed_time,$ds_overflow,$pebs_events,$pmc1_overflow,$cache_misses" >> ${PWD_OLD}/${b}.csv
    done
done
