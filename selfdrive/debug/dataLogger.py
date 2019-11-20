def logData(data):
  with open('/data/datalog.txt', 'a+') as f:
    for item in data:
      f.write("%s\t" % item)
    f.write("\n")

def logPid(pid_name,kp,ki,kf,set_point,measured,p,i,f,control):
  with open('/data/pidlog.txt', 'a+') as fil:
    fil.write("{0:.3f}\t{1:.3f}\t{2:.3f}\t{3:.3f}\t{4:.3f}\t{5:.3f}\t{6:.3f}\t{7:.3f}\t{8:.9f}\t{9:.3f}".\
      format(pid_name,kp,ki,kf,set_point,measured,p,i,f,control))
    fil.write("\n")
