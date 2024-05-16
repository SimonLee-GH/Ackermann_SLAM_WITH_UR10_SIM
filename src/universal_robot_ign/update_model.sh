#!/bin/bash
path=`pwd`
for dir in $path/resource/models/*; do
    echo "parse "$dir
    cd $dir
    if [ ! -f "$dir/model.sdf.xmacro" ]; then
        echo "model.sdf.xmacro does not exist."
        continue
    fi
    tempfile=`mktemp temp.XXXXXX`
    xmacro4sdf model.sdf.xmacro > $tempfile
    lines_num=`cat $tempfile | wc -l`
    if (($lines_num > 3 )) ;then
        cat $tempfile > model.sdf
    else
        cat $tempfile
    fi
    rm -f temp.*
done