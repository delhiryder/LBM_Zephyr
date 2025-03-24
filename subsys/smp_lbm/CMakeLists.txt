zephyr_library(mgmt_mcumgr_transport)

zephyr_library_sources_ifdef(CONFIG_MCUMGR_TRANSPORT_LBM
        smp_lbm.c
)