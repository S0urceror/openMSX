source profile.tcl
source symbols.tcl

set maxframeskip 100
#set throttle off
#after time 30 "set throttle on"

#symbols::load ../../../USB/drivers/UsbEthernet/dist/usbether.sym
#symbols::load ../../../USB/drivers/NextorUsbHost/dist/driver.sym
#symbol CHGBNK 0x7FD0
#profile::section_scope_bp frame 0xfd9f
#profile::section_irq_bp frame
#profile::section_scope_bp frame [symbol DO_UNAPI_ENTRY] { [pc_in_slot 3 2] }
#profile::section_scope_bp eth_in_status [symbol ETH_IN_STATUS] { [pc_in_slot 3 2] }
#profile::section_scope_bp eth_get_frame [symbol ETH_GET_FRAME] { [pc_in_slot 3 2] }
#profile::section_scope_bp get_bulk_in [symbol GET_BULK_IN_PACKET] { [pc_in_slot 3 2] }
#profile::section_scope_bp eth_send_frame [symbol ETH_SEND_FRAME] { [pc_in_slot 3 2] }
#profile::section_scope_bp get_netstat [symbol ETH_GET_NETSTAT] { [pc_in_slot 3 2] }
#profile::section_scope_bp chgbnk [symbol CHGBNK] { [pc_in_slot 1] }
#profile::section_scope_bp usb_unapi [symbol UNAPI_ENTRY] { [pc_in_slot 1] }
#profile::section_scope_bp drv_init [symbol DRV_INIT] { [pc_in_slot 1] }
#profile::section_scope_bp drv_rw [symbol DEV_RW] { [pc_in_slot 1] }
#profile::section_scope_bp drv_info [symbol DEV_INFO] { [pc_in_slot 1] }
#profile::section_scope_bp drv_status [symbol DEV_STATUS] { [pc_in_slot 1] }
#profile::section_scope_bp drv_lun [symbol LUN_INFO] { [pc_in_slot 1] }
#profile::profile_osd c

#set _file [open "logfile" w]
#debug set_watchpoint write_mem { 0x4000 0x7fff } {[watch_in_slot 1 X X]} {
#    puts $_file [format "%x - %x | %x : %d" [peek16 [reg SP]] [reg pc] $wp_last_address $wp_last_value]
#}