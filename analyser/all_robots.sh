#!/bin/bash
echo "Analysing all robots"
# Empty contents of export folder
rm -rf export/*
# Update filter and analyse
cp filters/_all_robots.yaml filter.yaml
python main.py -u repos -e packages -e analysis
python datamanage/db_exporter.py
# Move contents of export to data
rm -rf ../data/_all_robots/metrics
cp -rl export/metrics ../data/_all_robots
cp -a export/*.csv ../data/_all_robots
cp -a export/*.json ../data/_all_robots
