#!/bin/bash

echo "Virtual Com 0&1 initiallization..."
sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVIRT0 PTY,raw,echo=0,link=/dev/ttyVIRT1 &

echo "Virtual Com 2&3 initiallization..."
sudo socat -d -d PTY,raw,echo=0,link=/dev/ttyVIRT2 PTY,raw,echo=0,link=/dev/ttyVIRT3 &

echo "Add permission..."
sudo chmod 777 /dev/ttyVIRT* 