def logData(data):
  with open('/data/dataLog.txt', 'w+') as f:
    for item in data:
      f.write("%s\t" % item)
    f.write("\n")