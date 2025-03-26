/*
 * Copyright (c) 2024, Jamie McCrae
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <mgmt/mcumgr/transport/smp_internal.h>
#include <mgmt/mcumgr/transport/smp_reassembly.h>

#include <zephyr/smp_lbm/smp_lbm.h>

#include "smtc_modem_api.h"

#define LBM_STACK_ID 0 // Make sure this does not diverge from the application level value !
                // TODO: We should probably make this a configuration parameter to ensure everything just works !

LOG_MODULE_REGISTER(smp_lbm, CONFIG_MCUMGR_TRANSPORT_LBM_LOG_LEVEL);

static int smp_lbm_uplink(struct net_buf *nb);

static uint16_t smp_lbm_get_mtu(const struct net_buf *nb);

struct smp_transport smp_lbm_transport = {
	.functions.output = smp_lbm_uplink,
	.functions.get_mtu = smp_lbm_get_mtu,
};

#ifdef CONFIG_SMP_CLIENT
struct smp_client_transport_entry smp_lbm_client_transport = {
	.smpt = &smp_lbm_transport,
	.smpt_type = SMP_USER_DEFINED_TRANSPORT,
};
#endif

#ifdef CONFIG_MCUMGR_TRANSPORT_LBM_POLL_FOR_DATA
static struct k_thread smp_lbm_thread;
K_KERNEL_STACK_MEMBER(smp_lbm_stack, CONFIG_MCUMGR_TRANSPORT_LBM_POLL_FOR_DATA_STACK_SIZE);
K_FIFO_DEFINE(smp_lbm_fifo);

struct smp_lbm_uplink_message_t {
	void *fifo_reserved;
	struct net_buf *nb;
	struct k_sem my_sem;
};

static struct smp_lbm_uplink_message_t empty_message = {
	.nb = NULL,
};

static void smp_lbm_uplink_thread(void *p1, void *p2, void *p3)
{
	struct smp_lbm_uplink_message_t *msg;

	while (1) {
		msg = k_fifo_get(&smp_lbm_fifo, K_FOREVER);
		uint16_t size = 0;
		uint16_t pos = 0;

		if (msg->nb != NULL) {
			size = msg->nb->len;
		}

		while (pos < size || size == 0) {
			uint8_t *data = NULL;
			uint8_t data_size;
			uint8_t temp;
			uint8_t tries = CONFIG_MCUMGR_TRANSPORT_LBM_POLL_FOR_DATA_RETRIES;

            smtc_modem_get_next_tx_max_payload( LBM_STACK_ID, &data_size );

			if (data_size > size) {
				data_size = size;
			}

			if (size > 0) {
				if ((data_size + pos) > size) {
					data_size = size - pos;
				}

				data = net_buf_pull_mem(msg->nb, data_size);
			}

			while (tries > 0) {
				int rc;

				rc = smtc_modem_request_uplink(LBM_STACK_ID, CONFIG_MCUMGR_TRANSPORT_LBM_FRAME_PORT,
#if defined(CONFIG_MCUMGR_TRANSPORT_LBM_CONFIRMED_UPLINKS)
						  true,
#else
						  false,
#endif
						  data, data_size
						 );

				if (rc != 0) {
					--tries;
				} else {
					break;
				}
			}

			if (size == 0) {
				break;
			}

			pos += data_size;
		}

		/* For empty packets, do not trigger semaphore */
		if (size != 0) {
			k_sem_give(&msg->my_sem);
		}
	}
}
#endif

void smp_lbm_downlink(uint8_t port, uint16_t len, const uint8_t *hex_data)
{

    LOG_ERR("Lbm SMP downlink: port %d, len %d", port, len);

	if (port == CONFIG_MCUMGR_TRANSPORT_LBM_FRAME_PORT) {
#ifdef CONFIG_MCUMGR_TRANSPORT_LBM_REASSEMBLY
		int rc;

		if (len == 0) {
			/* Empty packet is used to clear partially queued data */
			(void)smp_reassembly_drop(&smp_lbm_transport);
		} else {
			rc = smp_reassembly_collect(&smp_lbm_transport, hex_data, len);

			if (rc == 0) {
				rc = smp_reassembly_complete(&smp_lbm_transport, false);

				if (rc) {
					LOG_ERR("Lbm SMP reassembly complete failed: %d", rc);
				}
			} else if (rc < 0) {
				LOG_ERR("Lbm SMP reassembly collect failed: %d", rc);
			} else {
				LOG_ERR("Lbm SMP expected data left: %d", rc);

#ifdef CONFIG_MCUMGR_TRANSPORT_LBM_POLL_FOR_DATA
				/* Send empty Lbm packet to receive next packet from server */
				k_fifo_put(&smp_lbm_fifo, &empty_message);
#endif
			}
		}
#else
		if (len > sizeof(struct smp_hdr)) {
			struct net_buf *nb;

			nb = smp_packet_alloc();

			if (!nb) {
				LOG_ERR("Lbm SMP packet allocation failure");
				return;
			}

			net_buf_add_mem(nb, hex_data, len);
			smp_rx_req(&smp_lbm_transport, nb);
		} else {
			LOG_ERR("Invalid Lbm SMP downlink");
		}
#endif
	} else {
		LOG_ERR("Invalid Lbm SMP downlink");
	}
}

static int smp_lbm_uplink(struct net_buf *nb)
{
	int rc = 0;

#ifdef CONFIG_MCUMGR_TRANSPORT_LBM_FRAGMENTED_UPLINKS
	struct smp_lbm_uplink_message_t tx_data = {
		.nb = nb,
	};

	k_sem_init(&tx_data.my_sem, 0, 1);
	k_fifo_put(&smp_lbm_fifo, &tx_data);
	k_sem_take(&tx_data.my_sem, K_FOREVER);
#else
	uint8_t data_size;
	uint8_t temp;

    smtc_modem_get_next_tx_max_payload( LBM_STACK_ID, &data_size );

	if (nb->len > data_size) {
		LOG_ERR("Cannot send Lbm SMP message, too large. Message: %d, maximum: %d",
			nb->len, data_size);
	} else {
		rc = smtc_modem_request_uplink(LBM_STACK_ID, CONFIG_MCUMGR_TRANSPORT_LBM_FRAME_PORT,
#if defined(CONFIG_MCUMGR_TRANSPORT_LBM_CONFIRMED_UPLINKS)
				  true,
#else
				  false,
#endif
                    nb->data, nb->len
				 );

		if (rc != 0) {
			LOG_ERR("Failed to send Lbm SMP message: %d", rc);
		}
	}
#endif

	smp_packet_free(nb);

	return rc;
}

static uint16_t smp_lbm_get_mtu(const struct net_buf *nb)
{
	ARG_UNUSED(nb);

	uint8_t max_data_size;
	uint8_t temp;

    smtc_modem_get_next_tx_max_payload( LBM_STACK_ID, &max_data_size );

	return (uint16_t)max_data_size;
}

static void smp_lbm_start(void)
{
	int rc;

	rc = smp_transport_init(&smp_lbm_transport);

#ifdef CONFIG_SMP_CLIENT
	if (rc == 0) {
		smp_client_transport_register(&smp_lbm_client_transport);
	}
#endif

#ifdef CONFIG_MCUMGR_TRANSPORT_LBM_REASSEMBLY
	smp_reassembly_init(&smp_lbm_transport);
#endif

#ifdef CONFIG_MCUMGR_TRANSPORT_LBM_POLL_FOR_DATA
	k_thread_create(&smp_lbm_thread, smp_lbm_stack,
			K_KERNEL_STACK_SIZEOF(smp_lbm_stack),
			smp_lbm_uplink_thread, NULL, NULL, NULL,
			CONFIG_MCUMGR_TRANSPORT_LBM_POLL_FOR_DATA_THREAD_PRIORITY, 0,
			K_FOREVER);

	k_thread_start(&smp_lbm_thread);
#endif
}

MCUMGR_HANDLER_DEFINE(smp_lbm, smp_lbm_start);
