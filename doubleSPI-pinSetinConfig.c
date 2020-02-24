#include "nrfx_spim.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>

static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);  /**< SPI instance. */

static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/**
//Constant Defines
#define TEST_STRING "Nordic123456789012345678901234567890"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. 
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. 

**/

//DDS Settings
const uint8_t DDSCHIPSELECT = 3; //Pin 14 on nRF52DK == pin 3 arduino - Find in variant.cpp
const uint8_t DDSSYNCIO = 8;     //Pin 19 == pin 8

//define values to send
#define DDSCONTROLREGISTER 0x00
#define DDSFTWREGISTER 0x04        //CHANGE FOR AD9913 to 0x03
static uint8_t DDSFTWREGISTER_buf[] = {DDSFTWREGISTER};
static uint8_t DDSCONTROLREGISTER_buf[] = {DDSCONTROLREGISTER};

const uint32_t LEFTFTW = 0x22222222; //10Mhz @ 75Mhz in
const uint32_t STRAIGHTFTW = 0x33333333; //15Mhz @ 75Mhz in
const uint32_t RIGHTFTW = 0x44444444; //20Mhz @ 75Mhz in
//

void spim_event_handler(nrfx_spim_evt_t const * p_event,
                       void *                  p_context)
{
    spi_xfer_done = true;
}

int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);


    nrfx_spim_xfer_desc_t DDSFTWREGISTER_desc = NRFX_SPIM_XFER_TX(DDSFTWREGISTER_buf, 1);
    nrfx_spim_xfer_desc_t DDSCONTROLREGISTER_desc = NRFX_SPIM_XFER_TX(DDSCONTROLREGISTER_buf, 1);


    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M; 
    spi_config.ss_pin         = SPI_SS_PIN;   //SPI pin configuration is all handled in sdk_config.h
    spi_config.miso_pin       = SPI_MISO_PIN;
    spi_config.mosi_pin       = SPI_MOSI_PIN;
    spi_config.sck_pin        = SPI_SCK_PIN;
    nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL);

    while (1)
    {
        // Reset rx buffer and transfer done flag
        spi_xfer_done = false;
        nrfx_spim_xfer(&spi, &DDSFTWREGISTER_desc, 0);
        while (!spi_xfer_done)
        {
            __WFE();
        }

        // Transmit again
        spi_xfer_done = false;
        nrfx_spim_xfer(&spi, &DDSCONTROLREGISTER_desc, 0);
        while (!spi_xfer_done)
        {
            __WFE();
        }

        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}
