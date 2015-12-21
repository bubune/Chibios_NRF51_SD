# List of all the NRF51 platform files.
PLATFORMSRC = ${PORTED}/os/hal/platforms/NRF51/hal_lld.c \
              ${PORTED}/os/hal/platforms/NRF51/pal_lld.c \
              ${PORTED}/os/hal/platforms/NRF51/serial_lld.c \
              ${PORTED}/os/hal/platforms/NRF51/softdevice_lld.c

# Required include directories
PLATFORMINC = ${PORTED}/os/hal/platforms/NRF51 \
              ${PORTED}/os/hal/platforms/NRF51/softdevice
