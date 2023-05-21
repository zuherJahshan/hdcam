from enum import Enum
import logging
import math
from typing import List, Iterable, Iterator

import serial_gateway as sgw
import get_usb_serial_port

Word = int # 64 bit word
HitRate = int

log = logging.getLogger('HDCAM')
log.setLevel(logging.DEBUG)
# create console handler and set level to debug
ch = logging.StreamHandler()
# ch.setLevel(logging.DEBUG)
# create formatter
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
# add formatter to ch
ch.setFormatter(formatter)
# add ch to logger
log.addHandler(ch)

def getWordBurst(word: int,
                 offset,
                 burst_size):
    return (word >> offset) & ((1 << burst_size) - 1)

class Controller (object):
    # For now, we only supports HDCAM mode (no LSH) were only one block is filled.
    
    def __init__(self,
                 block_number = 0):
        self.xbox: XBOX = XBOX(null_value=0x0)
        self.PowerSupplier = None
        self.number_of_blocks = 4
        self.capacity = 480 * self.number_of_blocks # 480 rows per bank, and there is 4 banks.
        self.block_number = block_number
        
    def read(self,
             words: List[Word]) -> HitRate:
        # Reads the words stated in the words list and returns the number of hits (i.e., the number of matching words)
        
        ######### low level method definitions #########
        def xboxWrites(words):
            # Itrator that iterates on the xbox words and returns the portion that should be written.
            base_idx = 0
            while (base_idx < len(words)):
                yield words[base_idx: base_idx + self.xbox.getCapacity()]
                base_idx += self.xbox.getCapacity()
        ################################################
        
        hit_rate = 0 # TODO: change abstraction level of hit_rate
        for xbox_write in xboxWrites(words):
            log.debug("This is another XBOX write")
            self.xbox.write(xbox_write)
            burst = math.ceil(len(xbox_write) / 4)
            hit_rate += self.xbox.triggerHDCAMReadEvent(burst)
        return hit_rate
    
    def write(self,
              words: List[Word]) -> None:
        # Writes at most <capcity> words inside the HDCAM memory, if there are more than <capacity>, then discards the rest.
        if self.capacity < len(words):
            log.warning('Given list of words is of size {}, which exceeds the HDCAM capacity = {}.\
                Discarding all words after the index {}.'.format(len(words), self.capacity, self.capacity))
        xbox_words_iter = Controller.ControllerToXboxWriteWordsListAdapter(
            words=words[:self.xbox.getCapacity()],
            block_number=self.block_number,
            number_of_blocks=self.number_of_blocks
        ).getWords()
        burst = self.xbox.write(xbox_words_iter)
        self.xbox.triggerHDCAMWriteEvent(burst)
        
    def setPowerSupplierConf(self):
        pass
    
    class ControllerToXboxWriteWordsListAdapter(object):
        def __init__(self,
                     words: List[Word],
                     block_number = 0,
                     number_of_blocks = 4,
                     null_value = 0x0):
            self.words: List[Word] = words
            self.size:                      int = 0 # TODO: compute size
            self.bulk_block_size:           int = 32
            self.block_number:              int = block_number
            self.number_of_blocks:          int = number_of_blocks # 
            self.null_value:                Word = null_value
        
        def getWords(self) -> Iterator[Word]:
            words_index = 0
            xbox_index = 0
            current_word = self.null_value
            # make it aligned for 4 words
            while words_index < len(self.words) or (words_index % self.number_of_blocks) != 0:
                if self._isBulkLine(xbox_index) or words_index >= len(self.words):
                    yield self.null_value
                # elif not self._indexInIntededBlock(xbox_index):
                #     yield current_word
                else:
                    yield self.words[words_index]
                    current_word = self.words[words_index]
                    words_index += 1
                xbox_index += 1
            
        ################## Private ################
        def _isBulkLine(self,
                        index):
            return self._getRowIndex(index) % self.bulk_block_size == 0 or\
                (self._getRowIndex(index) % self.bulk_block_size) == (self.bulk_block_size - 1)
            
        def _indexInIntededBlock(self,
                                 index):
            return self._getColumnIndex(index) == self.block_number
            
            
        def _getRowIndex(self,
                         index):
            return index / self.number_of_blocks
        
        def _getColumnIndex(self,
                            index):
            return index % self.number_of_blocks
    
    
class XBOX(object):
    def __init__(self,
                 null_value):
        self.xbox_mem_base_addr = XBOX.Address.MIN_XBOX
        self.null_value = null_value
        self.capacity: int = 2048 # of words which
        self.serial_gateway = self._defineSerialGateway()
        self._enableHDCAMIMXWrapper()
        self._setPowerGateOn()
        self.hdcam_ctrl = XBOX.HDCAMCtrl(self.serial_gateway,
                                         ctrl_register_in_addr = XBOX.Address.MIN_XBOX_REGS,
                                         result_register_out_addr = XBOX.Address.RESULT_REGISTER)
        self.words = [Word] * 512 * 4
    
    def clear(self):
        lsb = getWordBurst(word=self.null_value,
                               offset=0,
                               burst_size=32)
        msb = getWordBurst(word=self.null_value,
                            offset=32,
                            burst_size=32)
        for _ in range(self.capacity):
            xbox_mem_ptr = self.xbox_mem_base_addr
            self.serial_gateway.wr_mem_by_uart(xbox_mem_ptr, lsb)
            xbox_mem_ptr += 4
            self.serial_gateway.wr_mem_by_uart(xbox_mem_ptr, msb)
            xbox_mem_ptr += 4
    
    def write(self,
              xbox_words_iterator: Iterable[Word]) -> int: # Returns the Burst of 256 bit elements written
        words_num = 0
        log.debug('Writing to XBOX')
        xbox_mem_ptr = self.xbox_mem_base_addr
        for word in xbox_words_iterator:
            lsb = getWordBurst(word=word,
                               offset=0,
                               burst_size=32)
            msb = getWordBurst(word=word,
                               offset=32,
                               burst_size=32)
            self.serial_gateway.wr_mem_by_uart(xbox_mem_ptr, lsb)
            xbox_mem_ptr += 4
            self.serial_gateway.wr_mem_by_uart(xbox_mem_ptr, msb)
            xbox_mem_ptr += 4
            words_num += 1
            log.debug('data {}-{} written to address {}'.format(bin(msb), bin(lsb), xbox_mem_ptr - 8))
        return int(words_num / 4)
    
    def getCapacity(self) -> int:
        return self.capacity
    
    def triggerHDCAMReadEvent(self,
                              burst) -> HitRate:
        return self.hdcam_ctrl.readSync(burst)
    
    def triggerHDCAMWriteEvent(self,
                               burst) -> None:
        return self.hdcam_ctrl.writeSync(burst)
    
    ################ Private ##################
    class Address:
      GP_RF_BASE_ADDR = 0x1A301000
      GP_RF_POWER_SHUT_OFF_ADDR = (GP_RF_BASE_ADDR + 0x0007C)
      MIN_XBOX = 0x1A340000
      MIN_XBOX_REGS = (MIN_XBOX + 0x8000*7)
      RESULT_REGISTER = MIN_XBOX_REGS + 4
      S_VALUES_REGISTER = RESULT_REGISTER + 4
    
    def _defineSerialGateway(self):
        serial_port_name, _ = get_usb_serial_port.get_usb_serial_port()
        if (serial_port_name==None) :
            print("No Active USB Serial Port, Quitting\n")
            exit()          
        stdio_serial_gate = sgw.serGate()
        esc_serial_gate = sgw.serGate()
        serial_gateway = sgw.serGateWay(serial_port_name,stdio_serial_gate, esc_serial_gate)
        target_br = 72000
        if (serial_gateway.target_baudrate != target_br):
            print ("Changing target_baudrate to %d" % target_br)
            serial_gateway.target_baudrate = target_br
        serial_gateway.activate()
        while (serial_gateway.serPort.inWaiting()>0) : # Clear pending UART communication from device
            # ignoreChar = serial_gateway.serPort.read(1) 
            serial_gateway.gwIterate()
        return serial_gateway
  
    def _enableHDCAMIMXWrapper(self):
        self.serial_gateway.wr_mem_by_uart(XBOX.Address.MIN_XBOX_REGS+32*4, 1<<1)
            
    def _setPowerGateOn(self):
        self._wrField(XBOX.Address.GP_RF_POWER_SHUT_OFF_ADDR,3,1,0x0)
        
    def _wrField(self,
                 xbox_addr: int,
                 field_width: int,
                 field_lsb: int,
                 data: int):
        mask = 0xFFFFFFFF
        mask >>= (32-field_width)
        data &= mask
        mask <<= field_lsb
        data <<= field_lsb
        r = self.serial_gateway.rd_mem_by_uart(xbox_addr)
        r &= ~mask
        r |=  data
        self.serial_gateway.wr_mem_by_uart(xbox_addr, r)
        
    Register = int
    
    class HDCAMCtrl(object):
        def __init__(self,
                     serial_gateway,
                     ctrl_register_in_addr,
                     result_register_out_addr):
            self.ctrl_register = XBOX.HDCAMCtrl.CtrlRegister(serial_gateway,
                                                             ctrl_register_in_addr)
            self.result_register = XBOX.HDCAMCtrl.ResultRegister(serial_gateway,
                                                                 result_register_out_addr)
        
        def readSync(self,
                     burst) -> HitRate:
            self.ctrl_register.setOperationType(XBOX.HDCAMCtrl.CtrlRegister.OperationType.COMPARE)
            
             # TODO: When LSH support will be added this should be updated.
            self.ctrl_register.setDigitalThreshold(0)
            self.ctrl_register.setChipMode(XBOX.HDCAMCtrl.CtrlRegister.ChipMode.HDCAM)
            self.ctrl_register.setBurst(burst)
            
            # Execute the command in hdcam
            self.ctrl_register.writeRegInXbox()
            
            # poll untill result is back
            while self.ctrl_register.getOperationType() != XBOX.HDCAMCtrl.CtrlRegister.OperationType.IDLE:
                log.debug(self.ctrl_register.getOperationType())
            
            return self.result_register.getResult()
            
        def writeSync(self,
                      burst) -> None:
            self._initSync()
            log.debug("Started control register write")
            self.ctrl_register.setOperationType(XBOX.HDCAMCtrl.CtrlRegister.OperationType.WRITE)
            self.ctrl_register.setChipMode(XBOX.HDCAMCtrl.CtrlRegister.ChipMode.LSH4) # Insignificant
            self.ctrl_register.setBurst(burst)
            # Execute the command in hdcam
            self.ctrl_register.writeRegInXbox()
            
            # poll untill result is back
            while self.ctrl_register.getOperationType() != XBOX.HDCAMCtrl.CtrlRegister.OperationType.IDLE:
                log.debug(self.ctrl_register.getOperationType())
            
            return
        
        def _initSync(self) -> None:
            self.ctrl_register.setOperationType(XBOX.HDCAMCtrl.CtrlRegister.OperationType.INIT)
            # self.ctrl_register.setChipMode(XBOX.HDCAMCtrl.CtrlRegister.ChipMode.HDCAM) # redundant
            # self.ctrl_register.setBurst(0) # Redundant
            
            self.ctrl_register.writeRegInXbox()
            
            while self.ctrl_register.getOperationType() != XBOX.HDCAMCtrl.CtrlRegister.OperationType.IDLE:
                log.debug(self.ctrl_register.getOperationType())
                
            return
        
        
        ######### Private ###########
        class CtrlRegister(object):
            OperationType = Enum('OperationType', ['IDLE', 'WRITE', 'INIT', 'COMPARE'],
                                 start=0)
            ChipMode = Enum('ChipMode', ['LSH4', 'LSH2', 'HDCAM'],
                            start=0)
            
            def __init__(self,
                         serial_gateway,
                         ctrl_register_xbox_addr):
                self.serial_gateway = serial_gateway
                self.reg_data = 0
                self._InitRegData()
                self.ctrl_register_xbox_addr = ctrl_register_xbox_addr  
        
            def getOperationType(self) -> OperationType:
                reg_data = self.serial_gateway.rd_mem_by_uart(self.ctrl_register_xbox_addr)
                return XBOX.HDCAMCtrl.CtrlRegister.OperationType((reg_data >> 28) & 3)
            
            def setOperationType(self,
                                operation_type: OperationType) -> None:
                self.reg_data |= (XBOX.HDCAMCtrl.CtrlRegister.OperationType(operation_type).value << 28)
            
            def setBurst(self,
                         burst):
                assert burst <= 512
                self.reg_data |= burst
            
            def setChipMode(self,
                            chip_mode: ChipMode) -> None:
                self.reg_data |= (XBOX.HDCAMCtrl.CtrlRegister.ChipMode(chip_mode).value << 12)
            
            def setDigitalThreshold(self,
                                    digital_threshold) -> None:
                # Applies only if working in LSH mode.
                if digital_threshold >= 4:
                    log.error("Digital threshold most be smaller than 4.")
                self.reg_data |= (digital_threshold << 30)
            
            def writeRegInXbox(self) -> None:
                self.serial_gateway.wr_mem_by_uart(self.ctrl_register_xbox_addr, self.reg_data)
                self.reg_data = 0
                
            
            ########### Private ############
            def _InitRegData(self) -> None:
                self.reg_data = 0
            
            
        class ResultRegister(object):
            def __init__(self,
                         serial_gateway,
                         result_register_xbox_addr):
                self.serial_gateway = serial_gateway
                self.result_register_xbox_addr = result_register_xbox_addr
            
            def getResult(self) -> HitRate:
                reg_data = self.serial_gateway.rd_mem_by_uart(self.result_register_xbox_addr)
                log.debug(" Register data is: {}".format(reg_data))
                return (reg_data >> 16) & ((1 << 12) - 1)
