


#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "max30105.h"
//#include "max30105.c"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0



/* Mode for LM75B. */
#define NORMAL_MODE 0U

/* Indicates if operation on TWI has ended. */


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from  sensor. */


/**
 * @brief TWI events handler.
 */


/**
 * @brief UART initialization.
 */

/**
 * @brief Function for reading data from temperature sensor.
 */
//static void read_sensor_data()
//{
 //   m_xfer_done = false;

    /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
  //  ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
  //  APP_ERROR_CHECK(err_code);
//}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    NRF_LOG_INFO("\r\nTWI init finished.");
    NRF_LOG_FLUSH();
    //MAX30105_set_mode();
    NRF_LOG_INFO("\r\nMAX30105 set mode finished.");
    NRF_LOG_FLUSH();
    setup(6.4,4,3,50,411,16384);
    //my_timer_start();

    while (true)
    {
        nrf_delay_ms(500);

 
        int ir = getIR();
        NRF_LOG_INFO("IR : %d",ir);
        int red = getRed();
        NRF_LOG_INFO("RED : %d",red);
        int green = getGreen();
        NRF_LOG_INFO("GREEN : %d",green);

        NRF_LOG_FLUSH();
    }
}
/** @} */


