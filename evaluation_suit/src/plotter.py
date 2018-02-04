import sys
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


from os import listdir
from os.path import isfile, join

if len(sys.argv) < 2:
    print "usage %s: file_coverage.txt"%(sys.argv[0])
    sys.exit(0)


file = sys.argv[1]

with open(file) as stream:
    content = stream.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
content = [(float(x.strip().split(" ")[0]), float(x.strip().split(" ")[1])) for x in content]
#content = [float(x.strip().split(" ")[1]) for x in content] 
print content

# create the general figure     
plt.plot([x[0] for x in content], [x[1] for x in content])
plt.ylabel("% Coverage")
plt.xlabel("Views")
savefig(file + ".png")
plt.clf()

