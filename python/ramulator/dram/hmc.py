from ramulator.dram.spec import DRAMStandard, TimingConstraint


class HMC(DRAMStandard):
    name = "HMC"
    internal_prefetch_size = 8
    read_latency = "nCL + nBL"

    # One HMC device per vault; the vault count lives in the HMC memory system.
    levels = {
        "Channel":      "N_A",
        "Rank":         "N_A",
        "BankGroup":    "N_A",
        "Bank":         "Closed",
        "Row":          "Closed",
        "Column":       "N_A",
    }

    commands = [
        "ACT", "PREpb", "PREab",
        "RD", "WR", "RDA", "WRA",
        "REFab",
    ]

    states = ["Opened", "Closed", "N_A"]

    timing_params = [
        "rate", "nBL",
        "nCL", "nRCD", "nRP", "nRAS", "nRC",
        "nWR", "nRTP", "nCWL",
        "nCCDS", "nCCDL",
        "nRRDS", "nRRDL",
        "nWTRS", "nWTRL",
        "nFAW", "nRFC", "nREFI",
        "tCK_ps",
    ]

    supported_requests = {"Read": "RD", "Write": "WR"}

    timing_constraints = [
        TimingConstraint(level="Channel", preceding=["RD", "RDA"], following=["RD", "RDA"], latency="nBL"),
        TimingConstraint(level="Channel", preceding=["WR", "WRA"], following=["WR", "WRA"], latency="nBL"),

        TimingConstraint(level="Rank", preceding=["RD", "RDA"], following=["RD", "RDA"], latency="nCCDS"),
        TimingConstraint(level="Rank", preceding=["WR", "WRA"], following=["WR", "WRA"], latency="nCCDS"),
        TimingConstraint(level="Rank", preceding=["RD", "RDA"], following=["WR", "WRA"], latency="nCL + nCCDS + 2 - nCWL"),
        TimingConstraint(level="Rank", preceding=["WR", "WRA"], following=["RD", "RDA"], latency="nCWL + nBL + nWTRS"),
        TimingConstraint(level="Rank", preceding=["RD"], following=["PREab"], latency="nRTP"),
        TimingConstraint(level="Rank", preceding=["WR"], following=["PREab"], latency="nCWL + nBL + nWR"),
        TimingConstraint(level="Rank", preceding=["ACT"], following=["ACT"], latency="nRRDS"),
        TimingConstraint(level="Rank", preceding=["ACT"], following=["ACT"], latency="nFAW", window=4),
        TimingConstraint(level="Rank", preceding=["ACT"], following=["PREab"], latency="nRAS"),
        TimingConstraint(level="Rank", preceding=["PREab"], following=["ACT"], latency="nRP"),
        TimingConstraint(level="Rank", preceding=["PREpb", "PREab"], following=["REFab"], latency="nRP"),
        TimingConstraint(level="Rank", preceding=["REFab"], following=["ACT", "REFab"], latency="nRFC"),

        TimingConstraint(level="BankGroup", preceding=["RD", "RDA"], following=["RD", "RDA"], latency="nCCDL"),
        TimingConstraint(level="BankGroup", preceding=["WR", "WRA"], following=["WR", "WRA"], latency="nCCDL"),
        TimingConstraint(level="BankGroup", preceding=["WR", "WRA"], following=["RD", "RDA"], latency="nCWL + nBL + nWTRL"),
        TimingConstraint(level="BankGroup", preceding=["ACT"], following=["ACT"], latency="nRRDL"),

        TimingConstraint(level="Bank", preceding=["ACT"], following=["RD", "RDA", "WR", "WRA"], latency="nRCD"),
        TimingConstraint(level="Bank", preceding=["RD"], following=["PREpb"], latency="nRTP"),
        TimingConstraint(level="Bank", preceding=["WR"], following=["PREpb"], latency="nCWL + nBL + nWR"),
        TimingConstraint(level="Bank", preceding=["RDA"], following=["ACT"], latency="nRTP + nRP"),
        TimingConstraint(level="Bank", preceding=["WRA"], following=["ACT"], latency="nCWL + nBL + nWR + nRP"),
        TimingConstraint(level="Bank", preceding=["ACT"], following=["ACT"], latency="nRC"),
        TimingConstraint(level="Bank", preceding=["ACT"], following=["PREpb"], latency="nRAS"),
        TimingConstraint(level="Bank", preceding=["PREpb"], following=["ACT"], latency="nRP"),
    ]


def _org(bankgroup, bank, row, column=1 << 6, channel_width=32):
    return {"density": bankgroup * bank * row * column * channel_width >> 20,
            "dq": 32, "channel_width": channel_width, "rank": 1,
            "bankgroup": bankgroup, "bank": bank, "row": row, "column": column}


HMC.org_presets = {
    "HMC_4GB":         _org(4, 2, 1 << 16),
    "HMC_8GB":         _org(8, 2, 1 << 16),
    "HMC_4GB_bank16":  _org(4, 4, 1 << 15),
    "HMC_4GB_bank32":  _org(4, 8, 1 << 14),
    "HMC_4GB_bank64":  _org(4, 16, 1 << 13),
    "HMC_4GB_bank128": _org(4, 32, 1 << 12),
    "HMC_4GB_bank256": _org(4, 64, 1 << 11),
    "HMC_4GB_va64":    _org(4, 2, 1 << 15),
    "HMC_4GB_va128":   _org(4, 2, 1 << 14),
    "HMC_4GB_va256":   _org(4, 2, 1 << 13),
    "HMC_4GB_va512":   _org(4, 2, 1 << 12),
    "HMC_4GB_va1024":  _org(4, 2, 1 << 11),
}

# Ramulator 1 HMC model values (HMC 2.1 spec does not define DRAM core timings).
HMC.timing_presets = {
    "HMC_2500": {
        "rate": 2500, "nBL": 4,
        "nCL": 17, "nRCD": 17, "nRP": 17, "nRAS": 34, "nRC": 51,
        "nWR": 19, "nRTP": 9, "nCWL": 13,
        "nCCDS": 4, "nCCDL": 6,
        "nRRDS": 7, "nRRDL": 8,
        "nWTRS": 3, "nWTRL": 9,
        "nFAW": 17, "nRFC": 200, "nREFI": 9750,
        "tCK_ps": 800,
    },
    "HMC_2500_unlimit_bandwidth": {
        "rate": 2500, "nBL": 0,
        "nCL": 17, "nRCD": 17, "nRP": 17, "nRAS": 34, "nRC": 51,
        "nWR": 19, "nRTP": 9, "nCWL": 13,
        "nCCDS": 1, "nCCDL": 1,
        "nRRDS": 7, "nRRDL": 8,
        "nWTRS": 3, "nWTRL": 9,
        "nFAW": 17, "nRFC": 200, "nREFI": 9750,
        "tCK_ps": 800,
    },
}
