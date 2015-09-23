#!/bin/bash
for filename in filters/*
do
    name="$(basename $filename .yaml)"
    echo "Analysing $name..."
    # Empty contents of export folder
    rm -rf export/*
    # Update filter and analyse
    cp "$filename" filter.yaml
    python main.py -n -u source -e packages -e analysis
    python datamanage/db_exporter.py
    # Move contents of export to data
    rm -rf ../data/"$name"/metrics
    cp -rl export/metrics ../data/"$name"
    cp -a export/*.csv ../data/"$name"
    cp -a export/*.json ../data/"$name"
done
