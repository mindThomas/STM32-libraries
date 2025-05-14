#!/bin/bash

if [[ "$1" == "-q" ]]
then
    CLANG_FORMAT_FILES=$(git ls-files -m -o --exclude-standard | grep '.[tch]pp$\|\.h$\|\.cc$')
    if [[ ! -z ${CLANG_FORMAT_FILES} ]]
    then
        echo "[FORM] CLANG ${CLANG_FORMAT_FILES}"
        clang-format-6.0 -style=file -i -fallback-style=none ${CLANG_FORMAT_FILES}
    fi
    exit 0
fi

# Run clang-format an all *.h, *.c, *.cpp, *.cc and *.hpp files
ROOT=$(git rev-parse --show-toplevel)
find $ROOT -iname *.h -o -iname *.c -o -iname *.cpp -o -iname *.hpp -o -iname *.cc -o -iname *.tpp\
    | xargs clang-format-6.0 -style=file -i -fallback-style=none

# Format checks may run in parallel on differnt file types.
# To make sure git diff reports only relevant changes,
# we pipe it through the same find call as above.
find $ROOT -iname *.h -o -iname *.c -o -iname *.cpp -o -iname *.hpp -o -iname *.cc -o -iname *.tpp\
    | xargs git diff -- >> clang_format.patch

# Return success if clang-format made no changes
if [ ! -s clang_format.patch ]
then
    echo "Code is formatted properly."
    rm clang_format.patch
    exit 0
fi

# Fail if any changes were made
echo "Please, format your code before submitting!"
echo "---------------------------------------------------"
echo
echo "Run clang-format on the following files:"
echo 
git ls-files -m '*.h' '*.hpp' '*.cpp' '*.tpp' '*.cc'
echo
echo "==================================================="
echo
echo "The following changes were made by clang-format:"
echo
cat clang_format.patch
rm clang_format.patch
echo
exit 1
