#!/bin/bash
echo "Scanning 10.24.29.x network..."
for i in {1..254}; do
    (ping -c 1 -W 1 10.24.29.$i > /dev/null 2>&1 && echo "10.24.29.$i is UP") &
done
wait
echo "Scan complete."