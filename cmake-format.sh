#!/bin/bash

if [[ "$1" == "-q" ]]
then
    CMAKE_FORMAT_FILES=$(git ls-files -m -o --exclude-standard | grep '.CMakeLists.txt$\|\.cmake$')
    #to make cmake happy since cmake-format rewrite the file on each run
    array=($(echo "$CMAKE_FORMAT_FILES" | tr '\n' '\n'))
    for i in "${!array[@]}"
    do
        cfile=${array[i]}
        echo "[FORM] CMAKE $cfile"
        mkdir -p "/tmp/${cfile%/*}"
        cmake-format $cfile > /tmp/$cfile
        cdiff=$(diff $cfile /tmp/$cfile)
        if [[ ! -z $cdiff ]]
        then
            cp /tmp/$cfile $cfile
        fi
        rm /tmp/$cfile
    done
    exit 0
fi

# Run cmake-format an all CMakeLists.txt and *.cmake files
ROOT=$(git rev-parse --show-toplevel)
find $ROOT -iname CMakeLists.txt -o -iname *.cmake \
    | xargs cmake-format -i

# Format checks may run in parallel on differnt file types.
# To make sure git diff reports only relevant changes,
# we pipe it through the same find call as above.
find $ROOT -iname CMakeLists.txt -o -iname *.cmake \
    | xargs git diff -- >> cmake_format.patch

# Return success if cmake-format made no changes
if [ ! -s cmake_format.patch ]
then
    echo "CMake files are formatted properly."
    rm cmake_format.patch
    exit 0
fi

# Fail if any changes were made
echo "Please, format your cmake files before submitting!"
echo "---------------------------------------------------"
echo
echo "Run cmake-format on the following files:"
echo
git ls-files -m '*/CMakeLists.txt' '*.cmake'
echo
echo "==================================================="
echo
echo "The following changes were made by cmake-format:"
echo
cat cmake_format.patch
rm cmake_format.patch
echo
exit 1
