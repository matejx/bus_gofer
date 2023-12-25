#!/usr/bin/python

import sys,serial

def atcmd(cmnd, resp):
  #print cmnd
  ser.flushInput()
  ser.write(cmnd + '\n')
  r = ser.readline().rstrip()
  if (len(resp) > 0) and (r.find(resp) == -1):
    if r == '': r = '(none)'
    raise Exception('Error! expected ' + resp + '\ncmnd was: ' + cmnd + '\nresp was: ' + r + '\n')
  #print r
  return r

if len(sys.argv) < 5:
  print 'usage: rdo.py serial_if device_type length filename'
  print
  print 'device types: i2cee,spiee,spifls'
  sys.exit(1)

devtype = sys.argv[2].upper()
flen = int(sys.argv[3])
fn = sys.argv[4]
pgr_size = 256

ser = serial.Serial(sys.argv[1], 4*115200, timeout=0.5)
try:
  for i in range(5):
    try:
      atcmd('AT+BUFRDDISP=1', 'OK')
      break
    except:
      pass
  if i == 4:
    print 'proggy not responding'
    sys.exit(1)
    
  f = open(fn, 'wb')
  try:
    a = 0
    while a < flen:
      nb = min(pgr_size, flen - a)
      adr = hex(a)[2:].zfill(6)
      a = a + nb
      try:
        print adr,nb
        s = atcmd('AT+' + devtype + 'RD=' + adr + ',' + str(nb),'').decode('hex')
        if len(s) != nb:
          raise Exception('Error! Expected ' + str(pgr_size) + ' bytes, received ' + str(len(s)))
        f.write(s)
      except Exception, e:
        print e
        break
  finally:
    f.close()
finally:
  ser.close()

print 'Done.'
