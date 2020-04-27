import datetime
import subprocess
from os import mkdir
from os.path import isdir

import numpy as np

if __name__ == "__main__":
    interHelloIntervals = [5000]
    mobileNodes = np.arange(10, 40, 10)
    propagationRanges = [100]
    scenarioSide = 500
    rngSeeds = [123456]
    simulationTime = 20

    resultsDir = '.Results/'
    if (not isdir(resultsDir)):
        mkdir(resultsDir)

    animationsDir = '.Animations/'
    if (not isdir(animationsDir)):
        mkdir(animationsDir)

    outFile = resultsDir + "Simulation" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    with open(outFile, 'w+') as out:
        out.write("Hello Resolution (Milliseconds);Mobile Nodes;Propagation Range (Meters);Area (Square Meters);Seed;Sent Packets;Received Packets\n")
        out.close()

    outFile = "./scratch/" + outFile
    for helloResolution in interHelloIntervals:
        for nodes in mobileNodes:
            for propagationRange in propagationRanges:
                for seed in rngSeeds:
                    print("\n\nHello Granularity (Milliseconds): {}\nNumber of Nodes: {}\nPropagation Range (Meters): {}\nScenario Side Length (Meters): {}\nSeed: {}\n".format(helloResolution, nodes, propagationRange, scenarioSide, seed))
                    subprocess.call(["../waf", "--run", "DisasterRecovery --helloInterval={} --mobileNodes={} --outFile={} --propagationRange={} --rngSeed={} --scenarioSide={} --simulationTime={}".format(helloResolution, nodes, outFile, propagationRange, seed, scenarioSide, simulationTime)])