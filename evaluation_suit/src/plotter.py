
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


from os import listdir
from os.path import isfile, join

onlyfiles = [f for f in listdir(".") if isfile(f)]
onlyfiles = [f for f in onlyfiles if f.find("_coverage.txt") != -1]

for file in onlyfiles:

    with open(file) as stream:
        content = stream.readlines()
    # you may also want to remove whitespace characters like `\n` at the end of each line
    ascissa = [float(x.strip().split(" ")[0]) for x in content] 
    content = [float(x.strip().split(" ")[1]) for x in content] 

    # create the general figure     
    plt.plot(ascissa, content)
    plt.ylabel("% Coverage")
    plt.xlabel("Distance Threshold")
    savefig(file + ".pdf")
    plt.clf()
