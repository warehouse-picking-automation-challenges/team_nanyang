import serial
import time
import thread
import sys, optparse
import math

class USB2Dynamixel_Device():
    def __init__( self, dev_name = '/dev/ttyUSB0', baudrate = 57600 ):
        try:
            self.dev_name = string.atoi( dev_name )
        except:
            self.dev_name = dev_name 
        self.mutex = thread.allocate_lock()
        self.servo_dev = None
        self.acq_mutex()
        self._open_serial( baudrate )
        self.rel_mutex()

    def acq_mutex(self):
        self.mutex.acquire()

    def rel_mutex(self):
        self.mutex.release()

    def send_serial(self, msg):
        self.servo_dev.flushInput()	
        self.servo_dev.write( msg )

    def read_serial(self, nBytes=1):
        rep = self.servo_dev.read( nBytes )
        return rep

    def _open_serial(self, baudrate):
        try:
            self.servo_dev = serial.Serial(self.dev_name, baudrate, timeout=1.0)
            self.servo_dev.close()  
            self.servo_dev.setParity('N')
            self.servo_dev.setStopbits(1)
            self.servo_dev.open()
            self.servo_dev.flushOutput()
            self.servo_dev.flushInput()

        except (serial.serialutil.SerialException), e:
            raise RuntimeError('lib_rob: Serial port not found!\n')
        if(self.servo_dev == None):
            raise RuntimeError('lib_rob: Serial port not found!\n')

class Robotis_Servo():
    def __init__(self, USB2Dynamixel, servo_id, series = 'MX' ):
        self.series = series;
        self.return_delay = 250 * 2e-6
        if series == 'MX':		
            defaults = {
                'home_encoder': 0x7FF,
                'max_encoder': 0xFFF,
                'rad_per_enc': math.radians(360.0) / 0xFFF, 
                'max_ang': math.radians(180),
                'min_ang': math.radians(-180),
                'flipped': False,
                'max_speed': 100
                }

        self.dyn = USB2Dynamixel
        self.dyn.servo_dev.flush()
        self.servo_id = servo_id
        try:
            self.read_address(3)
        except Exception as inst:
            raise RuntimeError('lib_rob: Errored. No ID (%d) on bus (%s), or wrong position.\n' %
                               ( servo_id, self.dyn.dev_name ))
        data = self.read_address( 0x05, 1)
        self.return_delay = data[0] * 3e-6
        self.settings = {}
        for key in defaults.keys():
            if self.settings.has_key( key ):
                pass
            else:
                self.settings[ key ] = defaults[ key ]	

    def init_cont_turn(self):
        self.write_address(0x06, [0x00,0x00])
        time.sleep(0.25)
        self.write_address(0x08, [0x00,0x00])
        time.sleep(0.25)


    def init_multi_turn(self):
        self.write_address(0x06, [0xFF,0x0F])
        time.sleep(0.25)
        self.write_address(0x08, [0xFF,0x0F])
        time.sleep(0.25)

    def multi_turn_pos(self, a):
        if a < 0:
            a = 65535 + a
        hi,lo = a / 256, a % 256
        return self.write_address( 0x1e, [lo,hi] )

    def kill_cont_turn(self):
        self.write_address(0x06, [0x00,0x00])
        time.sleep(0.25)
        max_encoder = self.settings['max_encoder']
        hi,lo = max_encoder / 256, max_encoder % 256
        self.write_address(0x08, [lo,hi])

    def is_moving(self):
        data = self.read_address( 0x2e, 1 )
        return data[0] != 0

    def read_voltage(self):
        data = self.read_address( 0x2a, 1 )
        return data[0] / 10.

    def read_temperature(self):
        data = self.read_address( 0x2b, 1 )
        return data[0]

    def read_load(self):
        data = self.read_address( 0x28, 2 )
        load = data[0] + data[1] * 256
        if load>1024:
            return 1024-load
        else:
            return load

    def read_current(self):
        if self.series=='MX':
            data = self.read_address( 0x44, 2 )	#current spans addresses 0x44 and 0x45
            curr = data[0] + data[1] * 256
            return 4.5*(curr-2048)		#in mA
        else:
            return 0.

    def read_speed(self):
        data = self.read_address( 0x26, 2 )
        speed = data[0] + data[1] * 256
        if speed>1024:
            return float(1024-speed)/1024.0
        else:
            return float(speed)/1024.0

    #both moving speed in joint mode (between designated positions) as well as wheel mode (which only sets direction in operation)
    def apply_speed(self,amnt):
        amnt = max(0.,min(abs(amnt),1.0))
        speed_val = int(amnt*1023)
        if speed_val < 0:
            speed_val = speed_val+1024
        hi,lo = speed_val / 256, speed_val % 256
        self.write_address(0x20,[lo,hi])

    def read_encoder(self):
        data = self.read_address( 0x24, 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_target_encoder(self):
        data = self.read_address( 0x1e, 2 )
        enc_val = data[0] + data[1] * 256
        return enc_val

    def read_angle(self):
        ang = (self.read_encoder() - self.settings['home_encoder']) * self.settings['rad_per_enc']
        if self.settings['flipped']:
            ang = ang * -1.0
        return ang

 
    def read_max_torque(self):
        data = self.read_address( 0x0e, 2 )
        torque = data[0] + data[1] * 256
        return torque

    def apply_max_torque(self,val):
        amnt = max(0.,min(abs(val),1.0))
        n = int(amnt*1023)
        n = min(max(n,0), 1023)
        hi,lo = n / 256, n % 256
        self.write_address( 0x22, [lo,hi])
        return self.write_address( 0x0e, [lo,hi])

    def move_angle(self, ang, angvel=None, blocking=False):
        if angvel == None:
            angvel = self.settings['max_speed']
        if angvel > self.settings['max_speed']:
            print 'lib_robotis.move_angle: angvel too high - %.2f deg/s' % (math.degrees(angvel))
            print 'lib_robotis.ignoring move command.'
            return
        if ang > self.settings['max_ang'] or ang < self.settings['min_ang']:
            print 'lib_robotis.move_angle: angle out of range- ', math.degrees(ang)
            print 'lib_robotis.ignoring move command.'
            return
        
        self.set_angvel(angvel)

        if self.settings['flipped']:
            ang = ang * -1.0
        enc_tics = int(round( ang / self.settings['rad_per_enc'] ))
        enc_tics += self.settings['home_encoder']
        self.move_to_encoder( enc_tics )

        if blocking == True:
            while(self.is_moving()):
                continue

    def move_to_encoder(self, n):
        ''' move to encoder position n
        '''
        # In some border cases, we can end up above/below the encoder limits.
        #   eg. int(round(math.radians( 180 ) / ( math.radians(360) / 0xFFF ))) + 0x7FF => -1
        n = min( max( n, 0 ), self.settings['max_encoder'] ) 
        hi,lo = n / 256, n % 256
        return self.write_address( 0x1e, [lo,hi] )

    def read_goal(self):
        data = self.read_address( 0x1e, 2)
        enc_val = data[0] + data[1] * 256
        return enc_val

    #ADDED: enabling torque control mode for the MX-64 series and above ONLY:
    def enable_torque_mode(self):
        if self.series=='MX':
            return self.write_address(0x46, [1])
        else:
            return 0
    def disable_torque_mode(self):
        if self.series=='MX':
            return self.write_address(0x46, [0])
        else:
            return 0
    def apply_torque(self,amnt):
        if self.series=='MX':
            amnt = max(0.,min(abs(amnt),1.0))
            torque_val = int(amnt*1023)
            if torque_val < 0:
                torque_val = torque_val+1024
            hi,lo = torque_val / 256, torque_val % 256
            return self.write_address(0x47,[lo,hi])
        else:
            return 0

    #disabling/enabling address 18 effectively shuts down motor output
        #different from torque mode that's available in the MX models
        #USE ONLY AS ON/OFF quick shutdown control
    def enable_torque(self):
        return self.write_address(0x18, [1])

    def disable_torque(self):
        return self.write_address(0x18, [0])

    #same as apply_speed, except set rad/sec as opposed to single value scalar
    def set_angvel(self, angvel):
        ''' angvel - in rad/sec
        '''     
        rpm = angvel / (2 * math.pi) * 60.0
        angvel_enc = int(round( rpm / 0.111 ))
        if angvel_enc<0:
            hi,lo = abs(angvel_enc) / 256 + 4, abs(angvel_enc) % 256
        else:
            hi,lo = angvel_enc / 256, angvel_enc % 256
        
        return self.write_address( 0x20, [lo,hi] )

    def write_id(self, id):
        ''' changes the servo id
        '''
        return self.write_address( 0x03, [id] )

    def __calc_checksum(self, msg):
        chksum = 0
        for m in msg:
            chksum += m
        chksum = ( ~chksum ) % 256
        return chksum

    def ping(self):
        return self.read_address(self,0x01,nBytes=1)

    def read_address(self, address, nBytes=1):
        ''' reads nBytes from address on the servo.
            returns [n1,n2 ...] (list of parameters)
        '''
        msg = [ 0x02, address, nBytes ]
        return self.send_instruction( msg, self.servo_id )

    def write_address(self, address, data):
        ''' writes data at the address.
            data = [n1,n2 ...] list of numbers.
            return [n1,n2 ...] (list of return parameters)
        '''
        msg = [ 0x03, address ] + data
        return self.send_instruction( msg, self.servo_id )

    def send_instruction(self, instruction, id):
        time.sleep(self.return_delay)	#helps w/ communication consistency?

        msg = [ id, len(instruction) + 1 ] + instruction # instruction includes the command (1 byte + parameters. length = parameters+2)
        chksum = self.__calc_checksum( msg )
        msg = [ 0xff, 0xff ] + msg + [chksum]
        
        self.dyn.acq_mutex()
        try:
            self.send_serial( msg )
            data, err = self.receive_reply()
        except Exception as inst:
            self.dyn.rel_mutex()
            raise RuntimeError(repr(str(inst)))
        self.dyn.rel_mutex()
        
        if err != 0:
            self.process_err( err )
        
        return data

    def process_err( self, err ):
        raise RuntimeError('lib_robotis: An error occurred: %d\n' % err)

    def receive_reply(self):
        start = self.dyn.read_serial( 2 )	#from pydynamixel: possible that these contain empty bytes
        servo_id = self.dyn.read_serial( 1 )

        while servo_id=='\xff':
            servo_id = self.dyn.read_serial( 1 )	#on Sparkfun USB-to-RS485 chip, more than 3 header bytes are sometimes set - apparently not an issue with the USB2Dynamixel

        if type(servo_id) is not str or len(servo_id)!=1:
            raise RuntimeError('lib_robotis: Invalid message headers, got servo id of type: '+repr(type(servo_id))+' and length: '+repr(len(servo_id)))
        if ord(servo_id) != self.servo_id:
            raise RuntimeError('lib_robotis: Incorrect servo ID received')
        data_len = self.dyn.read_serial( 1 )
        err = self.dyn.read_serial( 1 )
        data = self.dyn.read_serial( ord(data_len) - 2 )
        checksum = self.dyn.read_serial( 1 ) 		#checksum is read but never compared...(by design, according to original lib_robotis.py)
        return [ord(v) for v in data], ord(err)
        

    def send_serial(self, msg):
        """ sends the command to the servo
        """
        out = ''
        for m in msg:
            out += chr(m)
        self.dyn.send_serial( out )

def find_servos(dyn):
    ''' Finds all servo IDs on the USB2Dynamixel '''
    print 'Scanning for Servos.'
    servos = []
    dyn.servo_dev.setTimeout( 0.03 ) # To make the scan faster
    for i in xrange(254):
        try:
            s = Robotis_Servo( dyn, i )
            print '\n FOUND A SERVO @ ID %d\n' % i
            servos.append( i )
        except:
            pass
    dyn.servo_dev.setTimeout( 1.0 ) # Restore to original
    return servos


def recover_servo(dyn):
    ''' Recovers a bricked servo by booting into diagnostic bootloader and resetting '''
    raw_input('Make sure only one servo connected to USB2Dynamixel Device [ENTER]')
    raw_input('Disconnect power from the servo, but leave USB2Dynamixel connected to USB. [ENTER]')

    dyn.servo_dev.setBaudrate( 57600 )
    
    print 'Get Ready.  Be ready to reconnect servo power when I say \'GO!\''
    print 'After a second, the red LED should become permanently lit.'
    print 'After that happens, Ctrl + C to kill this program.'
    print
    print 'Then, you will need to use a serial terminal to issue additional commands.',
    print 'Here is an example using screen as serial terminal:'
    print
    print 'Command Line:  screen /dev/robot/servo_left 57600'
    print 'Type: \'h\''
    print 'Response: Command : L(oad),G(o),S(ystem),A(pplication),R(eset),D(ump),C(lear)'
    print 'Type: \'C\''
    print 'Response:  * Clear EEPROM '
    print 'Type: \'A\''
    print 'Response: * Application Mode'
    print 'Type: \'G\''
    print 'Response:  * Go'
    print
    print 'Should now be able to reconnect to the servo using ID 1'
    print
    print
    raw_input('Ready to reconnect power? [ENTER]')
    print 'GO!'

    while True:
        s.write('#')
        time.sleep(0.0001)


if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-d', action='store', type='string', dest='dev_name',
                 help='Required: Device string for USB2Dynamixel. [i.e. /dev/ttyUSB0 for Linux, \'0\' (for COM1) on Windows]')
    p.add_option('--scan', action='store_true', dest='scan', default=False,
                 help='Scan the device for servo IDs attached.')
    p.add_option('--recover', action='store_true', dest='recover', default=False,
                 help='Recover from a bricked servo (restores to factory defaults).')
    p.add_option('--ang', action='store', type='float', dest='ang',
                 help='Angle to move the servo to (degrees).')
    p.add_option('--ang_vel', action='store', type='float', dest='ang_vel',
                 help='angular velocity. (degrees/sec) [default = 50]', default=50)
    p.add_option('--id', action='store', type='int', dest='id',
                 help='id of servo to connect to, [default = 1]', default=1)
    p.add_option('--baud', action='store', type='int', dest='baud',
                 help='baudrate for USB2Dynamixel connection [default = 57600]', default=57600)

    opt, args = p.parse_args()

    if opt.dev_name == None:
        p.print_help()
        sys.exit(0)

    dyn = USB2Dynamixel_Device(opt.dev_name, opt.baud)

    if opt.scan:
        find_servos( dyn )

    if opt.recover:
        recover_servo( dyn )

    if opt.ang != None:
        servo = Robotis_Servo( dyn, opt.id )
        servo.move_angle( math.radians(opt.ang), math.radians(opt.ang_vel) )
    
