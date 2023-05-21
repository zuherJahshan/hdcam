import serial.tools.list_ports

def get_usb_serial_port() :

    ports = serial.tools.list_ports.comports()

    usb_ser_ports_list = []

    print ("\nList of active serial ports:\n") 

    for port, desc, hwid in sorted(ports):    
       print("port:%s ; desc:%s ; hwid:%s" % (port, desc, hwid))
       if (desc.find("USB Serial Port")!=-1) or (desc.find("TTL-234X-3V3")!=-1) :
         pnx_fpga_present = (hwid.find("FT2QRKVPA")!=-1)
         if pnx_fpga_present :  # Skip Pulpenix FPGA Board connected in parallel
            print("NOTICE: detected Pulpenix FPGA Board connected in parallel")
         else :
            print ("%s OK" % desc)
            usb_ser_ports_list.append(port)
            
            
       
    if (len(usb_ser_ports_list)==0) :    
       print ("Sorry, Can't locate a USB serial Port")
       return None
    elif len(usb_ser_ports_list)==1 :
       usb_ser_port = usb_ser_ports_list[0]
    else:
       usb_ser_port = usb_ser_ports_list[-2] # On LEO/SANSA board in-case of more than one port , then the one before the last seems to be the relevant. 
    print ("\nFound USB Serial Port at %s\n" % usb_ser_port)           
    return (usb_ser_port,pnx_fpga_present)

#-------------------------------------------------


if __name__ == '__main__':   
    get_usb_serial_port()