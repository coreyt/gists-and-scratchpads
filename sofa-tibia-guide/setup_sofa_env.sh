#!/bin/bash
# Source this file to set up SOFA environment variables
# Usage: source setup_sofa_env.sh

export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux
export PYTHONPATH=$SOFA_ROOT/plugins/SofaPython3/lib/python3/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$SOFA_ROOT/lib:$LD_LIBRARY_PATH

echo "SOFA environment configured:"
echo "  SOFA_ROOT=$SOFA_ROOT"
echo "  PYTHONPATH includes SofaPython3"
echo "  LD_LIBRARY_PATH includes SOFA libs"
