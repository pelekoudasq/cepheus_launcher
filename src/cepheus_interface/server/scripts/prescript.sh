#!/bin/bash
SCRIPT=$1
ssh cepheus@cepheus.local "bash -s" < $SCRIPT
