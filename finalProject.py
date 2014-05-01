from numpy import *
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import os
import time
import sys

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def MatrixCreate(height, width):
    matrix = zeros( (height, width) )
    return matrix

# %%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def VectorCreate(size):
    vector = zeros( (size), dtype = 'f' )
    return vector

# %%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def MatrixRandomize(v):
    for i in range(0, v.shape[0]):
        for j in range(0,v[0].size):
            v[i, j] = random.uniform(-1.0, 1.0)
        
    return v

# %%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def MatrixPerturb(p, prob):
    c = p.copy()

    for i in range(0, p.shape[0]):
        for j in range(0, p[0].size):
            if (random.random() < prob):
                c[i, j] = random.uniform(-1.0, 1.0)
    
    return c

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def Update(neuronValues, weights, i):
    temp = 0
    for j in range(0, neuronValues[0].size):
        #temp = 0
        for k in range(0, neuronValues[0].size):
            temp = temp + (weights[j, k] * neuronValues[i-1, k])
        #--end k loop
            
        #print 'temp = ', temp
        if temp < 0:
            temp = 0
        elif temp > 1:
            temp = 1

        neuronValues[i, j] = temp
    #--end j loop

    return neuronValues

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def MeanDistance(actual, desired):
    distances = VectorCreate(10)
    
    for i in range(0, actual.size):
        x = abs(desired[ i ] - actual[ i ])
        distances[ i ] = x

    
    #print '\nactual', actual
    #print '\n\ndistances', distances

    mean = distances.mean()
    #print '\nMean Distance = ', mean

    return mean

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def PlotImage(matrix, figNum):
    plt.figure(figNum)
    plt.imshow(matrix, cmap = cm.gray, aspect = 'auto', interpolation = 'nearest')
    plt.show()

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def Fitness3Get(synapses):
    weightsFileName = 'weights.dat'
    fitFileName = 'fits.dat'

    SendSynapseWeightsToFile(synapses,weightsFileName)

    os.system('./build/products/Debug/AppRagdollDemo')

    while (os.path.isfile(fitFileName) == False):
        time.sleep(0.2)

    fitness = FitnessCollectFromFile(fitFileName)
    
    os.remove(fitFileName)
    os.remove(weightsFileName)

    #print fitness
    return fitness

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def SendSynapseWeightsToFile(synapses, weightsFileName):
    synapseFile = open(weightsFileName, 'w')

    #synapseFile.write(str(synapses))

    for synapse in synapses:
        for weight in synapse:
            synapseFile.write(str(weight) + "\n")
    synapseFile.close()

#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def FitnessCollectFromFile(fitFileName):
    fitnessFile = open(fitFileName, 'r')

    fitness = float(fitnessFile.read())

    return fitness


#%%%%%&&&&&&&&&&&%%%%%%&&&&&&&&&&&%
def Evolve(numSensors, numMotors, numGenerations):
    fits = MatrixCreate(1, numGenerations)

    parent = MatrixCreate(numSensors, numMotors)
    parent = MatrixRandomize(parent)
    #print 'parent', parent
    parentFitness = Fitness3Get(parent)


    for generation in range(0, numGenerations):
        fits[0, generation] = parentFitness

        child = MatrixPerturb(parent, 0.05)
        childFitness = Fitness3Get(child)

        print generation,parentFitness,childFitness
        
        if ( childFitness > parentFitness ):
            parent = child
            parentFitness = childFitness

    return fits

    
# %%%%%&&&&&&&&&&&%^^^^%&&&%^^^^^^^^%%%%%%%%%%%%%%&&&&&&&&&&&7

numSensors = 4
numMotors = 11
numGenerations = 1000

fits = Evolve(numSensors, numMotors, numGenerations)
