# List of the ChibiOS/RT Cortex-M0 NRF51 port files.
PORTSRC = $(CHIBIOS)/os/ports/GCC/ARMCMx/crt0.c \
          $(PORTED)/os/ports/GCC/ARMCMx/NRF51/vectors.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/chcore.c \
          ${CHIBIOS}/os/ports/GCC/ARMCMx/chcore_v6m.c \
          ${CHIBIOS}/os/ports/common/ARMCMx/nvic.c

PORTASM =

PORTINC = ${CHIBIOS}/os/ports/common/ARMCMx/CMSIS/include \
          ${CHIBIOS}/os/ports/common/ARMCMx \
          ${CHIBIOS}/os/ports/GCC/ARMCMx \
          ${PORTED}/os/ports/GCC/ARMCMx/NRF51

PORTLD  = ${PORTED}/os/ports/GCC/ARMCMx/NRF51/ld
