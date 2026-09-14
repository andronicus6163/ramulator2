import math

from ramulator.dram.hbm3 import HBM3


class HBM3E(HBM3):
    """HBM3E: vendor extension of JESD238 HBM3 (24 Gb dies, 8/12-high, ~9.6 Gb/s pins).

    Interface, commands and constraints are inherited from HBM3. There is no JEDEC HBM3E
    datasheet; all timings below are guesstimates.
    """

    name = "HBM3E"

    @staticmethod
    def _resolve_nCCDR(num_sids, nCCDS):
        if num_sids == 1:
            return nCCDS
        return {
            # === Ramulator Guesstimate ===
            2: 3,
            3: 3,
            4: 3,
            # =============================
        }.get(num_sids, -1)

    @staticmethod
    def _resolve_nRFC(die_density, stack_height, channel_density, tCK_ps):
        tRFC_ns = {
            # === Ramulator Guesstimate (interpolated from JESD238 Table 93) ===
            (24576, 8, 12288): 400,
            (24576, 12, 18432): 460,
            # =============================
        }.get((die_density, stack_height, channel_density))
        if tRFC_ns is None:
            return -1
        return math.ceil(tRFC_ns * 1000 / tCK_ps)

    @staticmethod
    def _resolve_nRFCpb(die_density, stack_height, tCK_ps):
        # === Ramulator Guesstimate (JESD238 16 Gb/die value) ===
        return math.ceil(200 * 1000 / tCK_ps)


HBM3E.org_presets = {
    # One preset is one 64-bit channel split into two 32-bit pseudo-channels; 16 channels per stack.
    # 24 Gb/die: 8-high = 24 GB stack, 12-high = 36 GB stack. Row and SID counts are not powers of two.
    "HBM3E_24Gb_8hi":  {"die_density": 24576, "channel_density": 12288, "stack_height": 8,  "dq": 32, "channel_width": 64, "pseudochannel": 2, "sid": 2, "bankgroup": 4, "bank": 4, "row": 24576, "column": (1 << 5) << 3},
    "HBM3E_24Gb_12hi": {"die_density": 24576, "channel_density": 18432, "stack_height": 12, "dq": 32, "channel_width": 64, "pseudochannel": 2, "sid": 3, "bankgroup": 4, "bank": 4, "row": 24576, "column": (1 << 5) << 3},
}

HBM3E.timing_presets = {
    # tCK = 4 / 9.6 Gb/s. Analog timings keep the HBM3_6400Mbps values in ns, re-quantised to 417 ps.
    "HBM3E_9600Mbps": {
        "rate": 9600, "nBL": 2,
        "nCCDS": 2,
        # === Ramulator Guesstimate ===
        "nCL": 30, "nRCDRD": 47, "nRCDWR": 23,
        "nRP": 39, "nRAS": 68, "nWR": 50,
        "nRTP": 14, "nCWL": 15,
        "nRRDS": 6, "nRRDL": 8, "nFAW": 36,
        "nWTRS": 11, "nWTRL": 15,
        # =============================
        "nPPD": 2, "tCK_ps": 417,
    },
}
