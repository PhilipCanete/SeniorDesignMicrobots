#include "nrfx_spim.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include   <string.h>

//Function stub
void uint32_to_uint8_arr(uint8_t* arr, uint32_t val) {
  //Uses bit shifting to convert 32-bit int to array of 4 8-bit ints
  arr[3] = (uint8_t)(val & 0xFF);
  arr[2] = (uint8_t)((val >> 8) & 0xFF); 
  arr[1] = (uint8_t)((val >> 16) & 0xFF);
  arr[0] = (uint8_t)((val >> 24) & 0xFF);
}

void toggleHiLo(uint32_t GPIO) { // Make settings take effect.
  nrf_gpio_pin_set(GPIO);
  nrf_delay_ms(1);  // Prevents hold time violation
  nrf_gpio_pin_clear(GPIO);

}


//SPI Settings 
static const nrfx_spim_t spi = NRFX_SPIM_INSTANCE(0);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

//DDS SPI Settings
const uint8_t DDSCHIPSELECT = 3; //Pin 14 on nRF52DK == pin 3 arduino - Find in variant.cpp
const uint8_t DDSSYNCIO = 8;     //Pin 19 == pin 8

//DDS Values
#define DDSCONTROLREGISTER    0x00
#define DDSFTWREGISTER        0x04        //CHANGE FOR AD9913 to 0x03
#define LEFTFTW               0x22,0x22,0x22,0x22 //10Mhz @ 75Mhz in
#define STRAIGHTFTW           0x33,0x33,0x33,0x33 //15Mhz @ 75Mhz in
#define RIGHTFTW              0x44,0x44,0x44,0x44 //20Mhz @ 75Mhz in

static uint8_t DDSFTWREGISTER_buf[] = {DDSFTWREGISTER};
static uint8_t DDSCONTROLREGISTER_buf[] = {DDSCONTROLREGISTER};
static uint8_t LEFTFTW_buf[] = {LEFTFTW};
static uint8_t STRAIGHTFTW_buf[] = {STRAIGHTFTW};
static uint8_t RIGHTFTW_buf[] = {RIGHTFTW};

void spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context) {
    spi_xfer_done = true;
}

int main(void){

    //GPIO Initialization
    bsp_board_init(BSP_INIT_LEDS);
    
    nrf_gpio_cfg_output(DDSCHIPSELECT); //Set DDSCHIPSELECT GPIO as output
    nrf_gpio_pin_clear(DDSCHIPSELECT);  //Clear (set to low) GPIO DDSCHIPSELECT
    
    nrf_gpio_cfg_output(DDSSYNCIO);
    nrf_gpio_pin_clear(DDSSYNCIO);


    //SPI DMA register setting
    nrfx_spim_xfer_desc_t DDSFTWREGISTER_desc = NRFX_SPIM_XFER_TX(DDSFTWREGISTER_buf, 1);
    nrfx_spim_xfer_desc_t DDSCONTROLREGISTER_desc = NRFX_SPIM_XFER_TX(DDSCONTROLREGISTER_buf, 1);
    nrfx_spim_xfer_desc_t LEFTFTW_desc = NRFX_SPIM_XFER_TX(LEFTFTW_buf, 4);
    nrfx_spim_xfer_desc_t STRAIGHTFTW_desc = NRFX_SPIM_XFER_TX(STRAIGHTFTW_buf, 4);
    nrfx_spim_xfer_desc_t RIGHTFTW_desc = NRFX_SPIM_XFER_TX(RIGHTFTW_buf, 4);

    //SPI initiation
    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M; 
    spi_config.ss_pin         = SPI_SS_PIN;   //SPI pin configuration is all handled in sdk_config.h
    spi_config.miso_pin       = SPI_MISO_PIN;
    spi_config.mosi_pin       = SPI_MOSI_PIN;
    spi_config.sck_pin        = SPI_SCK_PIN;
    nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL);

    //Superloop
    while (1){
        // Reset rx buffer and transfer done flag
        spi_xfer_done = false;
        nrfx_spim_xfer(&spi, &DDSFTWREGISTER_desc, 0);
        while (!spi_xfer_done){
            __WFE();
        }

        // Transmit DDSCONTROLREGISTER_desc
        spi_xfer_done = false;
        nrfx_spim_xfer(&spi, &DDSCONTROLREGISTER_desc, 0);
        while (!spi_xfer_done){
            __WFE();
        }

        // Transmit DDSFTWREGISTER_desc
        spi_xfer_done = false;
        nrfx_spim_xfer(&spi, &DDSFTWREGISTER_desc, 0);
        while (!spi_xfer_done){
            __WFE();
        }
        
        // Transmit LEFTFTW_desc
        spi_xfer_done = false;
        nrfx_spim_xfer(&spi, &LEFTFTW_desc, 0);
        while (!spi_xfer_done){
            __WFE();
        }

        //LED stuff - only for debugging
        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}
