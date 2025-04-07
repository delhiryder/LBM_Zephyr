//
// Created by sidd on 3/24/25.
//

#ifndef LBM_ZEPHYR_SMP_LBM_H
#define LBM_ZEPHYR_SMP_LBM_H

void smp_lbm_downlink(uint8_t port, uint16_t len, const uint8_t *hex_data);
void smp_lbm_maybe_resend_uplink(void);
void smp_lbm_set_fuota_successful(bool successful);
void smp_lbm_set_confirmed_uplink_ack_received(bool received);

#endif //LBM_ZEPHYR_SMP_LBM_H
