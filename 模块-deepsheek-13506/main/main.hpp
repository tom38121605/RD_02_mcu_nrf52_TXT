#if !defined(MAIN_H)
#define MAIN_H

    #include "led_driver.hpp"
    #include "ommo_config.h"
    #include "ommocomm_uarte.hpp"
    #include "port_master.hpp"
    #include "utils.hpp"

    #define HOT_PLUG_EVENT_BIT	(0x80)
    #define HOT_PLUG_PKT_INTVL    100

    #define BASESTATION_LOST_TIMEOUT 20 //20*100ms = 2s
    #define OMMOCOMM_IDLE_PING_INTERVAL 5 //1/2s
    #define USB_DATA_SYNC_LOST_COUNT_TO_SYSTEM_ALERT 20

    typedef enum
    {
        DATA_MODE_DISABLED,
        DATA_MODE_USB,
        DATA_MODE_WIRELESS
    } data_mode_t;

    // Order is significant, see map_state_to_led_indication
    enum application_state
    {
        APP_STATE_CRASH           = 0,
        APP_STATE_POWER_DOWN      = 1,
        APP_STATE_FUEL_GAUGE_DFU  = 2,
        APP_STATE_CHARGING        = 3,
        APP_STATE_OMMOCOMM_DFU    = 4,
        APP_STATE_DC              = 5,
        APP_STATE_IDLE            = 6,
        APP_STATE_CONNECTED       = 7,
        APP_STATE_SYNCH_LOST      = 8,
        APP_STATE_SLEEP           = 9,
        APP_STATE_WIRELESS_SYNCH  = 10,
        APP_STATE_WIRED_SYNCH     = 11,
    };

    // Order is significant, see map_state_to_led_indication
    enum application_substates {
        APP_SUBSTATE_NORMAL           = 0,
        APP_SUBSTATE_PASSIVE_CHARGING = 1,
        APP_SUBSTATE_LOW_BATTERY      = 2,
    };

    uint16_t process_packet_received(uint8_t data[], uint16_t length, uint8_t response_buffer[], uint16_t response_buffer_size);
  
    void shutdown();
    void enter_ship_mode();
    void data_mode_enable(bool enable);
    void disable_data_mode();
    void change_led_state(application_state, application_substates substate = APP_SUBSTATE_NORMAL, bool immediate = true);

    void modify_power_status(bool add_item, power_req_t req);
    void check_power_requirements();
    bool ping_ommocomm_uarte(ommocomm_uarte * ommocomm);

#endif