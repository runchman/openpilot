def logData(data):
  with open('/data/datalog.txt', 'a+') as f:
    for item in data:
      f.write("%s\t" % item)
    f.write("\n")

def logPid(pid_name,kp,ki,kf,set_point,measured,p,i,f,control):
  with open('/data/pidlog.txt', 'a+') as fil:
    fil.write("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}".\
      format(pid_name,kp,ki,kf,set_point,measured,p,i,f,control))
    fil.write("\n")

def logStateChange(newState):
  with open('/data/datalog.txt', 'a+') as f:
    f.write("State Change -> %s", newState)
    f.write("\n")
  with open('/data/pidlog.txt', 'a+') as fil:
    fil.write("State Change -> %s", newState)
    fil.write("\n")
