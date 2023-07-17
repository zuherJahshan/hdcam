import serial_gateway as sgw
import get_usb_serial_port



STAT_DIV_BY_UPDATED_BIT             = 4
CTRL_DIV_BY_UPDATE_BIT              = 10
CTRL_DIV_BY_LSB                     = 0
CTRL_DIV_BY_WIDTH                   = 10
CTRL_CLK_SEL_BIT                    = 11
STAT_CLK0_SELECTED_BIT              = 5
STAT_CLK1_SELECTED_BIT              = 6

MIN_FLL                             = 0x1A302000
MIN_FLL_CTRL_APB_S                  = MIN_FLL

GP_RF_BASE_ADDR                     = 0x1A301000
GP_RF_FLL_PORTS_ACCESS_ADDR         = GP_RF_BASE_ADDR + 0x00010
GP_RF_PNX_CLOCK_CTRL_ADDR           = GP_RF_BASE_ADDR + 0x00020
PNX_CARP                            = GP_RF_PNX_CLOCK_CTRL_ADDR


glob_serial_gateway = None


def set_baudrate(
    serial_gateway,
    target_br
):
    if (serial_gateway.target_baudrate != target_br):
        print ("Changing target_baudrate to %d" % target_br)
        serial_gateway.target_baudrate = target_br
    serial_gateway.activate()


def get_serial_gateway():
    global glob_serial_gateway
    
    # If already exists, return it.
    if glob_serial_gateway:
        return glob_serial_gateway
    
    # Else, create serial_gateway and return it.
    serial_port_name, _ = get_usb_serial_port.get_usb_serial_port()
    if (serial_port_name==None) :
        print("No Active USB Serial Port, Quitting\n")
        exit()          
    stdio_serial_gate = sgw.serGate()
    esc_serial_gate = sgw.serGate()
    serial_gateway = sgw.serGateWay(serial_port_name,stdio_serial_gate, esc_serial_gate)
    set_baudrate(serial_gateway, 72000)
    while (serial_gateway.serPort.inWaiting()>0) : # Clear pending UART communication from device
        # ignoreChar = serial_gateway.serPort.read(1) 
        serial_gateway.gwIterate()
    glob_serial_gateway = serial_gateway
    return glob_serial_gateway

def wr_field(
    xbox_addr: int,
    field_lsb: int,
    field_width: int,
    data: int
):
    serial_gateway = get_serial_gateway()
    mask = 0xFFFFFFFF
    mask >>= (32-field_width)
    data &= mask
    mask <<= field_lsb
    data <<= field_lsb
    r = serial_gateway.rd_mem_by_uart(xbox_addr)
    r &= ~mask
    r |=  data
    serial_gateway.wr_mem_by_uart(xbox_addr, r)

    
char = int    


def wr_bit(
    address: int,
    bit_index: char,
    data: char
):
    serial_gateway = get_serial_gateway()
    data &= 0x1
    r: int = serial_gateway.rd_mem_by_uart(address)
    r &= ~(1<<bit_index)
    r |=  data<<bit_index
    serial_gateway.wr_mem_by_uart(address, data)
    
    
def rd_bit(
    address: int,
    bit_index: char
) -> char:
    serial_gateway = get_serial_gateway()
    data = serial_gateway.rd_mem_by_uart(address)
    data >>= bit_index
    data &= 0x1
    return data


def carp_get_div_by_updated(
    address: int
):
    return rd_bit(address + 4, STAT_DIV_BY_UPDATED_BIT)


def carp_set_div_by_update(
    addr: int,
    val: char
):
    wr_bit(addr, CTRL_DIV_BY_UPDATE_BIT, val)

    # wait for update
    while carp_get_div_by_updated(addr) != val:
        pass
    

def carp_set_divider(
    addr: int,
    div_by: char
):
    carp_set_div_by_update(addr, 0)
    wr_field(addr, CTRL_DIV_BY_LSB, CTRL_DIV_BY_WIDTH, div_by)
    carp_set_div_by_update(addr, 1)
    carp_set_div_by_update(addr, 0)
    

def reg_poll(
    address: int,
    data: int,
    mask: int
):
    serial_gateway = get_serial_gateway()

    # read data before comparing
    rd_data = serial_gateway.rd_mem_by_uart(address)
    rd_data_masked = rd_data & mask

    while rd_data_masked != data:
        rd_data = serial_gateway.rd_mem_by_uart(address)
        rd_data_masked = rd_data & mask
        

def CTRL(address):
    return address + 0x0


def STAT(address):
    return address + 0x4


def carp_get_selected_clock(
    addr: int
):
    if rd_bit(STAT, STAT_CLK0_SELECTED_BIT):
        return 0

    if rd_bit(STAT, STAT_CLK1_SELECTED_BIT):
        return 1

    # called while no clock selected
    return -1


def carp_clock_select(
    addr: int,
    clk_sel: char
):
    wr_bit(CTRL(addr), CTRL_CLK_SEL_BIT, clk_sel)
    
    # wait for update
    while carp_get_selected_clock(addr) != clk_sel:
        pass


def config_pll_rcg(
    target_fll_out_clk_hz: int,
    pnx_carp_divider: int
):

#   unsigned int rd_data_cfg0, rd_data_cfg1, rd_data_cfg2, rd_data_cfg3;
#   unsigned int new_val, target_mult_fact, dco_code, clock_div, loop_gain;
#   int          delta;

#    Read the PLL's ports status:
    serial_gateway = get_serial_gateway()
    rd_data_cfg0 = serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x0) # Read FLL status register
    rd_data_cfg1 = serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x4) # Read FLL control reg-I reset value
    rd_data_cfg2 = serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x8) # Read FLL control reg-II reset value
#   rd_data_cfg3 = rd32(MIN_FLL_CTRL_APB_S + 0xC); // Read FLL integrator register

#   /*========================================================================================================== \
#   | These two fields responsible for the closed loop DCO convergence:                                          |
#   | Higher DCO frequency ensures lower quantization error                                                      |
#   | Higher loop gain 2^(-loop_gain) speeds the convergence, but restraint might be needed to prevent overshoot |
#   |                                                                                                            |
#   \==========================================================================================================*/
    dco_code  = 0x01F4 # f_DCO ~ 2000 [MHz] (each bit ~4MHz)
    loop_gain = 0x8

    new_val = (rd_data_cfg2 & 0xFFFFFFF0) + loop_gain; # Change loop gain.
    serial_gateway.wr_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x8, new_val)
    serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x8) # Read to verify write

    new_val = rd_data_cfg1 | 0x80000000; # Switch to normal mode

#   /*========================================================================== \
#   | F_out calculations only relevant when DCO locks                            |
#   | Update multiplication factor (M) to achieve: FREQ = f_REF * M / 2^(D_fb-1) |
#   | Quantization error ~ f_DCO/f_ref - floor(f_DCO/f_ref)                      |
#   \===========================================================================*/

#   /*---------------------------------------------------------------------------------------------------------.
#   | Standard crystal oscillators suitable for UART standard baud-rates:   (original document ref)                                    |
#   | XTAL = 32768 [Hz]  ==> FREQ = 32.768K * M / 2^(D_fb-1) ~ 675MHz                                         |
#   |    > M = 20600 = 0x5078                                                                                  |
#   |    > D_fb = 0x1                                                                                          |
#   |    > D_lg = 0x8                                                                                          |
#   `---------------------------------------------------------------------------------------------------------*/

    # LEO2 Setting 
    ref_clk_hz = 32768 # 31250.0
  
    PNX_CARP_DIVIDER = 4
 
    clock_div = 0x1 ; # Default ; This is FLL programmed divider(not external carp divider)

    # extra bit Rounding method without float
    target_mult_fact_x2: int = int(2*(target_fll_out_clk_hz*(1<<(clock_div-1))/ref_clk_hz))
    target_mult_fact: int  =  int(target_mult_fact_x2/2) + target_mult_fact_x2 % 2

    new_val = (new_val & 0xFFFF0000) | target_mult_fact # Adjust target multiplier
    new_val = (new_val & 0xC3FFFFFF) | (clock_div<<26)  # Adjust D_fb to non-default value
    new_val = (new_val & 0xFC00FFFF) | (dco_code<<16)   # Adjust DCO frequency
    serial_gateway.wr_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x4, new_val)
    serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x4); # Read to verify write

  # Check that FLL lock is set
    reg_poll(MIN_FLL_CTRL_APB_S + 0x20, 0x1, 0x00000001)

    # Read FLL status register for actual multiplier value
    rd_data_cfg0 = serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x0)
    # delta = abs(0x01F4 - (rd_data_cfg0 & 0x0000FFFF)); 
    # delta = abs(target_mult_fact - (rd_data_cfg0 & 0x0000FFFF));//
    delta = int(target_mult_fact) - int(rd_data_cfg0 & 0x0000FFFF)
    if delta < 0:
        delta = -delta
    while delta > 50: # Seek convergence
        # Read FLL status register for actual multiplier value
        rd_data_cfg0 = serial_gateway.rd_mem_by_uart(MIN_FLL_CTRL_APB_S + 0x0)
        # delta = abs(0x01F4 - (rd_data_cfg0 & 0x0000FFFF)); 
        # delta = abs(target_mult_fact - (rd_data_cfg0 & 0x0000FFFF));//
        delta = int(target_mult_fact) - int(rd_data_cfg0 & 0x0000FFFF)
        if delta < 0:
            delta = -delta
        

    # Enable FLLOUT
    serial_gateway.wr_mem_by_uart(GP_RF_FLL_PORTS_ACCESS_ADDR , 0x00000001)
    # wr32(GP_RF_FLL_PORTS_ACCESS_ADDR, 0x00000001);


    carp_set_divider(PNX_CARP, PNX_CARP_DIVIDER)

    # Remove PNX_CARP bypass   
    carp_clock_select(PNX_CARP, 1)


#define target_pnx_clk_hz_const (1000000000.0/ATSPEED_CLK_PERIOD_NS) 
#define target_fll_out_clk_hz_const (int)((target_pnx_clk_hz_const * PNX_CARP_DIVIDER)+0.5)

def init_pll_rcg(clk_period):
    target_pnx_clk_hz_const = int(1000000000.0/clk_period)
    target_fll_out_clk_hz_const = int((target_pnx_clk_hz_const * PNX_CARP_DIVIDER) + 0.5)
    config_pll_rcg(target_fll_out_clk_hz_const , PNX_CARP_DIVIDER)
