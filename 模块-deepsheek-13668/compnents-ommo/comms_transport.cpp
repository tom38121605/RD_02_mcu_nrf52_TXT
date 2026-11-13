#include "comms_transport.hpp"

void comms_transport_interface::set_callbacks(comms_transport_on_rx_complete rx_cb, comms_transport_on_tx_complete tx_cb, void *p_context_g)
{
    rx_complete_callback = (rx_cb);
    tx_complete_callback = (tx_cb);
    p_context = (p_context_g);
}

comms_transport_interface::~comms_transport_interface() = default;