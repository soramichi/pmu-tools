BUFFER_SIZE=1048576 # 1MB == (page_size << 8) == page_size * 256
COUNTER="0x20D1"
OUTPUT_MODE=0
TARGET="/home/soramichi/src/cifar/random_forest/cifar_rf.py"
PYTHON="/home/soramichi/anaconda3/bin/python"
RANGE="500 400 300 200 100"

# never edit these
PWD_OLD=`pwd` # Note: do not use name $PWD, as it's automatically overwritten by the shell
TARGET_DIR=`dirname $TARGET`

for r in $RANGE; do
    echo $r

    # insert the module
    insmod ./simple-pebs.ko
    ./init $COUNTER $r $BUFFER_SIZE $OUTPUT_MODE
    
    # run the workload. cd to $TARGET_DIR in case $TARGET has something
    # specified in a relative path, such as open("./data")
    cd $TARGET_DIR
    $PYTHON $TARGET > $PWD_OLD/$r.log

    # remove the module and save dmesg
    cd $PWD_OLD
    rmmod simple-pebs
    dmesg | tail -n 4 > $r.dmesg
done
