#BUFFER_SIZE=1048576 # 1MB == (page_size << 8) == page_size * 256
#BUFFER_SIZE=4194304 # 4MB
COUNTER="0x1c2"
OUTPUT_MODE=0
TARGET="runspec"
TARGET_ARGS="--config test.cfg --nobuild --noreportable --size=train --iterations=1 h264ref"
RESET_VALUES="2000 4000 8000 16000 32000 64000 128000"
BUFFER_SIZES="4194304"
#BUFFER_SIZES="65536"

# never edit these
PWD_OLD=`pwd` # Note: do not use name $PWD, as it's automatically overwritten by the shell
#TARGET_DIR=`dirname $TARGET`
TARGET_DIR="/home/soramichi/src/spec_cpu_2006"
TARGET="$TARGET $TARGET_ARGS"

cd $TARGET_DIR
. ./shrc
cd $PWD_OLD

for i in 1 2 3; do
    for r in $RESET_VALUES; do
	for b in $BUFFER_SIZES; do
	    mkdir -p ${b}/${i}
	    echo "BUFFER_SIZE: $b, RESET_VALUE: $r"

	    # insert the module
	    insmod ./simple-pebs.ko
	    ./init $COUNTER $r $b $OUTPUT_MODE

	    # run the workload. cd to $TARGET_DIR in case $TARGET has something
	    # specified in a relative path, such as open("./data")
	    cd $TARGET_DIR
	    $TARGET > ${PWD_OLD}/${b}/${i}/${r}.log

	    # remove the module and save dmesg
	    cd $PWD_OLD
	    rmmod simple-pebs
	    dmesg | tail -n 20 > ${b}/${i}/${r}.dmesg
	done
    done
done
