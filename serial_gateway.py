
import time
import serial
import sys

#================================================================================

SU_CMD_WR_WORD = 0x80   
SU_CMD_RD_WORD = 0x83  
SU_CMD_RSP     = 0x88 

#================================================================================

def configPort(portName, baudrate) :

   # configure the serial connections (the parameters differs on the device you are connecting to)
   ser = serial.Serial()
   ser.port=portName
   ser.baudrate = baudrate  
   ser.parity=serial.PARITY_NONE
   ser.stopbits=serial.STOPBITS_ONE
   ser.bytesize = serial.EIGHTBITS #number of bits per bytes
   ser.xonxoff = False             #disable software flow control
   ser.rtscts = False              #disable hardware (RTS/CTS) flow control
   ser.dsrdtr = False              #disable hardware (DSR/DTR) flow control
   ser.timeout = 1                 # non-block read    
   ser.writeTimeout = 1            #timeout for write  
   return ser   

#---------------------------------------------------------------------------------

class serGate () :

   def __init__(self):   
      self.fromPulpQue = [] 
      self.toPulpQue = []    

#---------------------------------------------------------------------------------


class serGateWay () :
  def __init__(self,serPortName,stdioSerGate,escSerGate):

    self.dflt_hw_baudrate_at_25Mhz = 72000 ; # DONT TOUCH! determined by hardwired default uart clock counter (346 for baudrate of 72000 at external clock of 25MHz) 

    self.target_baudrate = 72000 # 115200 # 72000 # 3500000
  
    self.serPort = configPort(serPortName, self.dflt_hw_baudrate_at_25Mhz)      
    self.stdioSerGate = stdioSerGate
    self.escSerGate = escSerGate  

    self.GPP_BASE = 0x1a300000
    self.CORE_CNTRL_REG_ADDR = self.GPP_BASE
    self.CORE_CNTRL_SW_JTAG_SEL_BIT_IDX = 0    
    self.CORE_CNTRL_SW_RST_BIT_IDX = 4
    self.CORE_CNTRL_EN_CORE_BIT_IDX = 8
    
    self.UART_BASE_ADDR = 0x1A100000       
    self.UART_REG_DLL   = self.UART_BASE_ADDR + 0x00 # Divisor Latch (LS)
    self.UART_REG_DLM   = self.UART_BASE_ADDR + 0x04 # Divisor Latch (MS)
    self.UART_REG_FCR   = self.UART_BASE_ADDR + 0x08 # FIFO Control Register (Write Only)      
    self.UART_REG_LCR   = self.UART_BASE_ADDR + 0x0c # Line  Control Register
    
    self.current_clk_div_cntr = None ;

    
  
  def activate(self) :
        
       self.serPort.open() 
              
  def gwIterate(self) :

           # Continues Read attempt from Serial Port (transmitted from  Pulp's TX)
           
           numSerPortInWaiting = self.serPort.inWaiting()
           if (numSerPortInWaiting>0) :           
              serReadCharsQ = self.serPort.read(numSerPortInWaiting)
 
              bi = 0
              while bi<len(serReadCharsQ) :
                  serReadCharByte = serReadCharsQ[bi]
                  serReadChar = chr(serReadCharByte)                  
                  if (serReadCharByte==SU_CMD_RSP) :
                     if (numSerPortInWaiting<5):                 
                         serReadCharsQ = serReadCharsQ + self.serPort.read(5-numSerPortInWaiting)                 
                     rdata = 0 
                     bi=bi+1                     
                     for i in range(4) :
                          rdata = rdata*256 + serReadCharsQ[bi]
                          bi=bi+1
                     self.escSerGate.fromPulpQue.append(rdata)                  
                  else :
                     self.stdioSerGate.fromPulpQue.append(serReadCharByte)
                  
                     if serReadCharByte > 127 :  
                       print("PY Serial Gateway : WARNING, Non ASCII: stdioSerGate.fromPulpQue.append(%d)" % ord(serReadChar))
                     bi=bi+1


                 
         # Continues write attempt from write Queues to Serial (To Pulp's RX)

           while len(self.escSerGate.toPulpQue) >= 2 :
             escCmd = self.escSerGate.toPulpQue.pop(0)
             escAddr = self.escSerGate.toPulpQue.pop(0)
             self.serPort.write(bytearray([escCmd]))
             self.serPort.write(escAddr.to_bytes(4, byteorder='big'))              
             if (escCmd==SU_CMD_WR_WORD) :
                while len(self.escSerGate.toPulpQue)==0 :
                  pass
                escWrData = self.escSerGate.toPulpQue.pop(0)  
                
                self.serPort.write(escWrData.to_bytes(4, byteorder='big'))

           while len(self.stdioSerGate.toPulpQue) > 0 : 
                stdioWrChar = self.stdioSerGate.toPulpQue.pop(0)                
                self.serPort.write(bytearray([stdioWrChar]))
                
    
  #-------------------------------------------------------------------------------------
  
  def wr_mem_by_uart(self,addr,wdata) :
    self.escSerGate.toPulpQue.append(SU_CMD_WR_WORD)            
    self.escSerGate.toPulpQue.append(addr)
    self.escSerGate.toPulpQue.append(wdata)         
    self.gwIterate()
  
  #-------------------------------------------------------------------------------------

  def queue_wr_mem_by_uart(self,addr,wdata) :
  
    # Same as above but without gwIterate
    # To be used in case we want to queue multiple accesses and submit at once 
    # which apparently is much faster for FTDI avoiding the port access overhead.
  
    self.escSerGate.toPulpQue.append(SU_CMD_WR_WORD)            
    self.escSerGate.toPulpQue.append(addr)
    self.escSerGate.toPulpQue.append(wdata)         
      
  #-------------------------------------------------------------------------------------

  def send_queue_wr_mem_by_uart(self) :
    # releases to pulp escape queue created by queue_wr_mem_by_uart
    CHUNK_SIZE = 5000 # to avoid time outs 

    byte_vec = bytearray()
    while len(self.escSerGate.toPulpQue) >= 2 :
      escCmd = self.escSerGate.toPulpQue.pop(0)
      escAddr = self.escSerGate.toPulpQue.pop(0)
      byte_vec.extend(bytearray([escCmd]))
      byte_vec.extend(escAddr.to_bytes(4, byteorder='big'))              
      if (escCmd==SU_CMD_WR_WORD) :
         while len(self.escSerGate.toPulpQue)==0 :
           pass
         escWrData = self.escSerGate.toPulpQue.pop(0)           
         byte_vec.extend(escWrData.to_bytes(4, byteorder='big'))

      # print("DBG len(byte_vec)%d" % len(byte_vec))
      if len(byte_vec)>=CHUNK_SIZE :
         # print  ("PYSHELL send_queue_wr_mem_by_uart sending CHUNK %d bytes" % len(byte_vec))  
         self.serPort.write(byte_vec)
         byte_vec = []

    # Send remaining 
    # print  ("PYSHELL send_queue_wr_mem_by_uart sending remaining %d bytes" % len(byte_vec))  
    self.serPort.write(byte_vec)
    

  #-------------------------------------------------------------------------------------

  
  def rd_mem_by_uart(self,addr) :
       
       self.escSerGate.toPulpQue.append(SU_CMD_RD_WORD)            
       self.escSerGate.toPulpQue.append(addr) 
       
       while len(self.escSerGate.fromPulpQue)==0 : # iterate serGW till read data is back
          self.gwIterate() 
              
       rdata = self.escSerGate.fromPulpQue.pop(0) 
                                   
       return rdata
  
  #-------------------------------------------------------------------------------------
  
  def getSerChar(self) :
      # poll till byte is available
      while (self.serPort.inWaiting()==0) : 
         pass
      return self.serPort.read(1)

  #-------------------------------------------------------------------------------

  def send_str_to_serial(self,string) :
     self.stdioSerGate.toPulpQue.extend(string.encode())
     self.stdioSerGate.toPulpQue.extend(bytearray([0])) # Close string
     self.gwIterate()
     
  #-------------------------------------------------------------------------------

  # def clear_device_response(self) :
  # 
  #    # clear leftover communication from device
  #    while (self.serPort.inWaiting()>0) :
  #       ignoreChar = self.serPort.read(1)
  #       #self.gwIterate()

  #-------------------------------------------------------------------------------

