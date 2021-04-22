/*
   -------------------------------------------------------------------
   EmonESP Serial to Emoncms gateway
   -------------------------------------------------------------------
   Adaptation of Chris Howells OpenEVSE ESP Wifi
   by Trystan Lea, Glyn Hudson, OpenEnergyMonitor

   Modified to use with the CircuitSetup.us Split Phase Energy Meter by jdeglavina

   All adaptation GNU General Public License as below.

   -------------------------------------------------------------------

   This file is part of OpenEnergyMonitor.org project.
   EmonESP is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3, or (at your option)
   any later version.
   EmonESP is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with EmonESP; see the file COPYING.  If not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
*/

// See energy meter specific configuration in energy_meter.h
#define ENABLE_ENERGY_METER

// If USE_SERIAL_INPUT is defined, EmonESP will check the serial port for
// input. Only enable this if there is input expected on the serial port;
// otherwise it seems to read garbage data.
//#define USE_SERIAL_INPUT

#include "emonesp.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_eth.h"
#include "driver/spi_master.h"
#include "esp_task_wdt.h"
#include "web_server.h"
#include "ota.h"
#include "input.h"
#include "emoncms.h"
#include "mqtt.h"

#ifdef ENABLE_ENERGY_METER
#include "energy_meter.h"
#endif

#define ETH_SPI_CLOCK_MHZ 20
#define ETH_SPI_HOST SPI2_HOST
#define ETH_MISO_GPIO 12
#define ETH_MOSI_GPIO 13
#define ETH_SCLK_GPIO 14
#define ETH_CS_GPIO 15
#define ETH_INT_GPIO 4

void eth_setup(void)
{
  tcpip_adapter_init();
  esp_event_loop_create_default();
  ESP_ERROR_CHECK(tcpip_adapter_set_default_eth_handlers());

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = 1;
  phy_config.reset_gpio_num = -1;

  gpio_install_isr_service(0);
  spi_device_handle_t spi_handle = NULL;
  spi_bus_config_t buscfg = {
      .mosi_io_num = ETH_MOSI_GPIO,
      .miso_io_num = ETH_MISO_GPIO,
      .sclk_io_num = ETH_SCLK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0,
      .flags = SPICOMMON_BUSFLAG_MASTER,
      .intr_flags = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, 1));
  spi_device_interface_config_t devcfg = {
      .command_bits = 1,
      .address_bits = 7,
      .dummy_bits = 0,
      .mode = 0,
      .duty_cycle_pos = 0,
      .cs_ena_pretrans = 0,
      .cs_ena_posttrans = 0,
      .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
      .input_delay_ns = 0,
      .spics_io_num = ETH_CS_GPIO,
      .flags = 0,
      .queue_size = 1,
      .pre_cb = NULL,
      .post_cb = NULL,
  };
  ESP_ERROR_CHECK(spi_bus_add_device(ETH_SPI_HOST, &devcfg, &spi_handle));
  /* dm9051 ethernet driver is based on spi driver */
  eth_dm9051_config_t dm9051_config = {
    .spi_hdl = spi_handle,
    .int_gpio_num = ETH_INT_GPIO,
  };
  esp_eth_mac_t *mac = esp_eth_mac_new_dm9051(&dm9051_config, &mac_config);
  esp_eth_phy_t *phy = esp_eth_phy_new_dm9051(&phy_config);

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t eth_handle = NULL;
  ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
//  uint8_t mac_addr[] = { 0x02, 0x00, 0x00, 0x12, 0x34, 0x56 };
//  ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));
  esp_eth_start(eth_handle);
}

static char input[MAX_DATA_LEN];
extern TaskHandle_t loopTaskHandle;

void arduinoTask(void *pvParameter) {
#ifdef ENABLE_WDT
  esp_task_wdt_add(loopTaskHandle);
#endif

  Serial.begin(115200);
#ifdef DEBUG_SERIAL1
  Serial1.begin(115200);
#endif
  delay(200);

  // Read saved settings from the config
  config_load_settings();
  delay(200);

  // Initialise the WiFi
  //wifi_setup();
  eth_setup();
  delay(200);

  // Bring up the web server
  web_server_setup();
  delay(200);

#ifdef ENABLE_WDT
  DBUGS.println("Watchdog timer is enabled.");
  esp_task_wdt_reset();
#endif

#ifdef ENABLE_WEB_OTA
  // Start the OTA update systems
  ota_setup();
#endif

#ifdef ENABLE_ENERGY_METER
  energy_meter_setup();
#endif

  DBUGLN("Server started");

  while(1) {
#ifdef ENABLE_WDT
    esp_task_wdt_reset();
#endif
    web_server_loop();
    wifi_loop();

#ifdef ENABLE_WEB_OTA
    ota_loop();
#endif

#ifdef ENABLE_ENERGY_METER
    energy_meter_loop();
#endif

    boolean gotInput = input_get(input);
    if (gotInput) {
      DBUGS.println(".");
    }

    if (wifi_client_connected()) {
      if (emoncms_apikey != 0 && gotInput) {
        //DBUGS.println(input);
        if (emoncms_server != 0) {
          emoncms_publish(input);
        }
      }
      if (mqtt_server != 0) {
        mqtt_loop();
        if (gotInput) {
          mqtt_publish(input);
        }
      }
    }
  }
}

extern "C" {
    void app_main(void);
}

void app_main()
{
    // initialize arduino library before we start the tasks
    initArduino();

    xTaskCreate(&arduinoTask, "arduino_task", 16384, NULL, 5, NULL);
}