# nRF5-freeRTOS-current-optimization

Example code for optimized the current consumption with FreeRTOS

## Example
* ble_app_hrs_freertos -- original native example from NRF5 SDK by enable the DCDC
* ble_app_nus_bas_dis_with_bonding_freeRTOS -- Nordic UART Service + Battery Service (with optimized SAADC measurement) + Device Information System with Peer Manager 
* ble_app_multiple_nus_c_freeRTOS -- BLE Multiple Central Role with NUS Client x 2 with FreeRTOS


## Requirement

* NRF52832 / NRF52840 DK Board
* Softdevice S132v6.1.1 or S140v6.1.1
* Power Profile Kit
* Nordic NRF5 SDK 15.2
* IDE : Segger Embedded Studio
