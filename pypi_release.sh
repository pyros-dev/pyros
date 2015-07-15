#!/bin/sh

#TODO : find ~/.pypirc and extract repository
if [ $# -lt 1 ]; then
    echo "Usage : $0 <pypi|testpypi>"
    exit 127
fi

#extract version from setup.py
VERSION=`python setup.py --version`

#if package index is "pypi" we fill in info into git
if [ "$1" = "pypi" ]; then
    GIT_TAG_CMD="git tag --VERSION"
    GIT_PUSH_CMD="git push --tags"
else
    GIT_TAG_CMD="echo git tag skipped when not releasing on main pypi index"
    GIT_PUSH_CMD="echo git push skipped when not releasing on main pypi index"
fi

#update package data
python setup.py register -r $1

#make sure version doesnt exist already
VERSION_EXISTS=`git tag | grep $VERSION | wc -l`

if [ $VERSION_EXISTS -eq 1 ]; then
    echo $VERSION already exists ! Aborting...
    exit 64
fi

#TODO : Make sure everything is commited already
#tagging new release
${GIT_TAG_CMD}

#doing bloom release
if [ "$1" = "pypi" ]; then
    bloom-release rostful_node --track indigo --rosdistro indigo
fi

#uploading package and pushing tags
python setup.py sdist upload -r $1 && ${GIT_PUSH_CMD}

#TODO : mix with bloom release to release both at the same time
# OR maybe we dont need a pypi package at all ??
