#!/bin/bash
SCRIPT=$1
ssh 192.198.1.2 "bash -s" < $SCRIPT
