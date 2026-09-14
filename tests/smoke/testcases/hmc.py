import ramulator

CONFIG = dict(
    dram_class="HMC",
    org_preset="HMC_4GB",
    timing_preset="HMC_2500",
    dram_kwargs={},
    controller_class="HMCVault",
    fast_ctrl_extra_kwargs=dict(
        refresh_manager=ramulator.refresh_manager.NoRefresh(),
    ),
    frontend_clock_ratio=4,
    stream_cls=8,
)
