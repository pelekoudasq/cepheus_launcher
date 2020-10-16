#!/bin/bash
SCRIPT=$1
ssh root@mrrobot.local "bash -s" < $SCRIPT
