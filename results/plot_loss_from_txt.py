
import re
import numpy as np 
import matplotlib.pyplot as plt

filename = 'v1-6000.txt'

 
def get_numbers(s, type = 'float'):
    if type == 'int':
        numbers = re.findall(r'\b\d+\b', s)
    elif type == 'float':
        numbers = re.findall(r"[-+]?\d*\.\d+|\d+", s)
    else:
        raise ValueError('get_numbers: wrong datatype setting')
    numbers = [float(s) for s in numbers]
    return numbers

def tupleList_to_multipleList(tl):
    '''
    { Explanation:
        Input tl = [(1, 2), (3, 4), (5, 6)]
        zip(*tl): [(1, 3, 5), (2, 4, 6)]
        map(list, zip(*tl)): a map object
        list(map(list, zip(*tl))): [[1, 3, 5], [2, 4, 6]]
    }
    '''
    return list(map(list, zip(*tl)))


# Get data from txt
contents = []
with open(filename, 'r') as f:
    while 1:
        line = f.readline()
        if not line:
            break
        if not (line[0]==' ' and line[1].isdigit()):
             continue
        # Process
        line = line[:-1]
        # print(line) # line format: 1: 848.762451, 848.762451 avg loss, 0.000000 rate, 4.364900 seconds, 64 images
        vals = get_numbers(line)
        # print(vals)
        contents.append(tuple(vals))

con = tupleList_to_multipleList(contents)
l_idx = con[0]
l_loss = con[1]
l_avg_loss = con[2]
l_rate = con[3]
l_time_cost = con[4]
l_images = con[5]

# Print
N = len(l_idx)
print("Reading {} lines".format(N))

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
ax.plot(l_idx, l_avg_loss, 'r')
ax.set_xlim([0, N])
ax.set_ylim([0, 2])
ax.set_title('loss history')
plt.xlabel('iterations (number of batches)')
plt.ylabel('loss')
plt.grid(True)
plt.savefig("loss history.png")
plt.show()
