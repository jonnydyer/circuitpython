USB_VID = 0x239A
USB_PID = 0x802A
USB_PRODUCT = "LYFT Bluetooth Badge"
USB_MANUFACTURER = "Jonny Dyer"

MCU_CHIP = nrf52840
SD ?= s140
SOFTDEV_VERSION ?= 6.1.0

BOOT_SETTING_ADDR = 0xFF000
BOOT_FILE = boards/$(BOARD)/bootloader/$(SOFTDEV_VERSION)/$(BOARD)_bootloader_$(SOFTDEV_VERSION)_s140

ifeq ($(SD),)
	LD_FILE = boards/nrf52840_1M_256k.ld
else
	LD_FILE = boards/adafruit_$(MCU_SUB_VARIANT)_$(SD_LOWER)_v$(firstword $(subst ., ,$(SOFTDEV_VERSION))).ld
	CIRCUITPY_BLEIO = 1
endif

NRF_DEFINES += -DNRF52840_XXAA -DNRF52840

QSPI_FLASH_FILESYSTEM = 1
EXTERNAL_FLASH_DEVICE_COUNT = 1
EXTERNAL_FLASH_DEVICES = "W25Q128JV_SQ"
