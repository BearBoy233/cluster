from matplotlib import pyplot as plt

start = 0 
filename_turtlebotPose = '/home/linux/work/dataset/saveFile/'+ 'turtlebotPose-2021-01-23-15-39-00'+'.txt'
turtlebotPose = [[],[],[],[]]
with open(filename_turtlebotPose, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("turtlebotPose-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        if linecnt == 1 :
            start = float(temp[0])
        turtlebotPose[0].append(float(temp[0]) - start)
        turtlebotPose[1].append(float(temp[1]))
        turtlebotPose[2].append(float(temp[2]))
        turtlebotPose[3].append(float(temp[3]))

print(start)

filename_uav1Pose = '/home/linux/work/dataset/saveFile/'+ 'uav1Pose-2021-01-23-15-39-00'+'.txt'
uav1Pose = [[],[],[],[]]
with open(filename_uav1Pose, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("uav1Pose-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        uav1Pose[0].append(float(temp[0]) - start)
        uav1Pose[1].append(float(temp[1]))
        uav1Pose[2].append(float(temp[2]))
        uav1Pose[3].append(float(temp[3]))

filename_trackerResult_uav1 = '/home/linux/work/dataset/saveFile/'+'trackerResult_uav1-2021-01-23-15-39-00'+'.txt'
trackerResult_uav1 = [[],[],[],[]]
with open(filename_trackerResult_uav1, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("trackerResult_uav1-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        trackerResult_uav1[0].append(float(temp[0]) - start)
        trackerResult_uav1[1].append(float(temp[1]) + float(temp[3])/2.0)
        trackerResult_uav1[2].append(float(temp[2]) + float(temp[4])/2.0)
        trackerResult_uav1[3].append(float(temp[5]))


fig = plt.figure(num=1) 
ax1 = fig.add_subplot(111)
ax1.plot(turtlebotPose[1], turtlebotPose[2], 'r', label='turtlebotPose')  
ax1.legend(loc='upper right')  
ax1.set_xlabel('x') 
ax1.set_ylabel('y') 

fig = plt.figure(num=2) 
ax1 = fig.add_subplot(111)
ax1.plot(trackerResult_uav1[1], trackerResult_uav1[2], 'r', label='image')  
ax1.legend(loc='upper right')  
ax1.set_xlabel('x') 
ax1.set_ylabel('y') 

fig = plt.figure(num=3) 
ax1 = fig.add_subplot(111)
ax1.plot(trackerResult_uav1[0], trackerResult_uav1[3], 'r', label='distance')  
ax1.legend(loc='upper right')  
ax1.set_xlabel('time') 
ax1.set_ylabel('distance') 

fig = plt.figure(num=4) 
ax1 = fig.add_subplot(111)
ax1.plot(uav1Pose[1], uav1Pose[2], 'r', label='uav1Pose')  
ax1.legend(loc='upper right')  
ax1.set_xlabel('x') 
ax1.set_ylabel('y') 

plt.show() 