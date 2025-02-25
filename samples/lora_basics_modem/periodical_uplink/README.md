# Periodical Uplink

This sample tries to join the network, then periodically sends uplinks.

It uses the (very new) "LBM main thread" that runs LBM outside of the main thread.
This will use the `/chosen/zephyr,lora-transceiver` device tree node.

It also sends uplinks when the USER button is pressed.

Build for nrf5340dk using:

```
west build -p -b nrf5340dk/nrf5340/cpuapp -- -DSHIELD=semtech_sx1262mb2cas
```
