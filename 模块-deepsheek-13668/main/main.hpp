#if !defined(MAIN_H)
#define MAIN_H

    #include <stdint.h>

    #define BASESTATION_LOST_TIMEOUT 20 //20*100ms = 2s

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
        APP_STATE_OMMOCOMM_DFU    = 1,
        APP_STATE_DC              = 2,
        APP_STATE_IDLE            = 3,
        APP_STATE_CONNECTED       = 4,
        APP_STATE_SYNCH_LOST      = 5,
        APP_STATE_SLEEP           = 6,
        APP_STATE_WIRELESS_SYNCH  = 7,
        APP_STATE_WIRED_SYNCH     = 8,
    };

    uint16_t fill_in_packet_id_request(uint8_t buffer[]);
    void data_mode_enable(bool enable);
    void change_led_state(application_state, bool immediate = true);
    void push_led_state(application_state);
    void pop_led_state();
    void sensors_disable();
    uint16_t process_packet_received(uint8_t data[], uint16_t packet_size, uint8_t response_buffer[], uint16_t response_buffer_size);

//--    uint16_t adc_scan_bus_generate_data_descriptor(uint8_t packet_id_request_buffer[], uint16_t buffer_size);

#endif