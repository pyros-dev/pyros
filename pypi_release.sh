#!/bin/sh

#TODO : find ~/.pypirc and extract repository
if [ $# -lt 1 ]; then
    echo "Usage : $0 <pypi|pypitest>"
    exit 127
fi

python setup.py register -r $1

python setup.py sdist upload -r $1
