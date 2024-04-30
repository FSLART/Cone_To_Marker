#!/usr/bin/env bash

URLS=(
    "https://github.com/FSLART/lart_msgs.git"
)
DIR='git_fetched'

mkdir -p $DIR

cd ./$DIR
for repo in ${URLS[@]}; do
  git clone $repo 2> /dev/null
done