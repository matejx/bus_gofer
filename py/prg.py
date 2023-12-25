#!/usr/bin/python

import sys,serial,datetime,hashlib,intelhex

SER_SPEED = 4800

def atcmd(cmnd, resp, to = 0.5):
  if ser.timeout != to:
    ser.timeout = to
  ser.flushInput()
  ser.write(cmnd + '\n')
  r = ser.readline().rstrip();
  if len(resp) > 0 and r.find(resp) == -1:
    if r == '': r = '(none)'
    raise Exception('Error! expected ' + resp + '\ncmnd was: ' + cmnd + '\nresp was: ' + r + '\n')
  return r

if len(sys.argv) < 4:
  print 'usage: prg.py serial_if device_type filename'
  print
  print 'device types: ee24, ee95, fls25'
  sys.exit(1)

devtype = sys.argv[2].upper()
if devtype == 'EE24': pgw_size = 64
elif devtype == 'EE95': pgw_size = 16
elif devtype == 'FLS25': pgw_size = 256
else:
  print 'unknown device type'
  sys.exit(1)

try:
  h = intelhex.IntelHex()
  if sys.argv[3][-3:] == 'hex':
    h.loadhex(sys.argv[3])
  else:
    h.loadbin(sys.argv[3])
except intelhex.HexReaderError, e:
  print "Bad hex file\n", str(e)
  exit(1)

ser = serial.Serial(sys.argv[1], SER_SPEED)
try:
  for i in range(5):
    try:
      atcmd('AT+BUFRDDISP=0', 'OK')
      break
    except:
      pass
  if i == 4:
    print 'bus gofer not responding'
    sys.exit(1)

  if devtype == 'FLS25':
    jid = atcmd('AT+FLS25ID', '')
    if jid == 'FFFFFF' or jid == '000000':
      print 'flash 25 not detected'
      exit(1)
    print 'JEDEC ID:',jid
    print 'Erasing flash. Wait...'
    atcmd('AT+FLS25CE', 'OK', 30)

  if devtype == 'EE24':
    atcmd('AT+I2CADR=A0', 'OK')

  hna = h.minaddr() # veeeeeery slow function
  hma = h.maxaddr() # same
  a = hna
  sdt = datetime.datetime.now()
  tb = 0
  md5 = hashlib.md5()
  while a < hma:
    s = ''
    adr = hex(a)[2:].zfill(6)
    nb = min(pgw_size, hma - a + 1)
    tb = tb + nb
    for i in range(nb):
      s = s + hex(h[a])[2:].zfill(2)
      a = a + 1
    md5.update(s.decode('hex'))
    try:
      te = datetime.datetime.now() - sdt # time elapsed
      pd = 1.0 * (a-hna) / (hma-hna) # prg done
      tl = datetime.timedelta(seconds = int(((1.0-pd)/pd) * te.total_seconds())) # time left
      print adr,nb,tb,'/',hma-hna+1,'(',int(100.0*pd),'%)',tl,' left'
      atcmd('AT+BUFWR=' + s, 'OK')
      atcmd('AT+' + devtype + 'WR=' + adr, 'OK')
      atcmd('AT+' + devtype + 'RD=' + adr + ',' + str(i + 1), 'OK')
      atcmd('AT+BUFCMP', 'OK')
    except Exception, e:
      print e
      break
  print
  if hna == 0:
    print 'Device MD5:',atcmd('AT+' + devtype + 'MD5=' + str(hma+1), '', 20)
    print 'File MD5  :',md5.hexdigest().upper()
finally:
  ser.close()

print 'Done.'
