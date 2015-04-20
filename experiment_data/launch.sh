#!/bin/bash -e

data="RABRange_5m"
maxnumjobs=8

# Create a data diretory
mkdir -p $data

for NormalBehav in SWARM_AGGREGATION SWARM_DISPERSION SWARM_HOMING
do
    for ErrorBehav in FAULT_NONE FAULT_STRAIGHTLINE FAULT_RANDOMWALK FAULT_CIRCLE FAULT_STOP
    do

    mkdir -p $data/${NormalBehav}/${ErrorBehav}

    done
done



for NormalBehav in SWARM_AGGREGATION SWARM_DISPERSION SWARM_HOMING; do
    for ErrorBehav in FAULT_NONE FAULT_STRAIGHTLINE FAULT_RANDOMWALK FAULT_CIRCLE FAULT_STOP; do
        for Replicates in $(seq 1 5); do
            # Take template.argos and make an .argos file for this experiment
            SUFFIX=${Replicates}${Replicates}${Replicates}
            sed -e "s/SEED/${Replicates}${Replicates}${Replicates}/"                    \
                -e "s/SWARM_BEHAVIOR/${NormalBehav}/"                   \
                -e "s/FAULT_BEHAVIOR/${ErrorBehav}/"                  \
                -e "s|DATAFILE|$data/${NormalBehav}/${ErrorBehav}/nohup_${SUFFIX}|" \
                template_epuck_hom_swarm.argos                       \
                > $data/${NormalBehav}/${ErrorBehav}/exp_${SUFFIX}.argos
            # Call ARGoS
            parallel --semaphore -j${maxnumjobs} argos3 -c $data/${NormalBehav}/${ErrorBehav}/exp_${SUFFIX}.argos &
        done
    done
done

sem --wait