uint8 version_h
uint8 version_l
uint8 bms_status
uint8 soc                  # SOC 0-100%
int32 current              # mA
uint16 cycle
int8[2] bq_ntc             # x1 degrees centigrade
int8[2] mcu_ntc            # x1 degrees centigrade
uint16[10] cell_vol        # cell voltage mV