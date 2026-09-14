import math

import pytest

import ramulator
import tests.device_timings.harness as device_timings


pytestmark = pytest.mark.device_timings

ORGS = ["HBM3E_24Gb_8hi", "HBM3E_24Gb_12hi"]


def make_dut(org, **overrides):
    dram = ramulator.dram.HBM3E(org_preset=org, timing_preset="HBM3E_9600Mbps", **overrides)
    return device_timings.DeviceUnderTest(dram)


def _addr(dut, *, sid, bankgroup, bank, row):
    return dut.addr_vec(PseudoChannel=0, Sid=sid, BankGroup=bankgroup, Bank=bank, Row=row, Column=0)


def test_hbm3e_org_capacity_matches_product_stacks():
    for org, stack_gb in (("HBM3E_24Gb_8hi", 24), ("HBM3E_24Gb_12hi", 36)):
        o = ramulator.dram.HBM3E.org_presets[org]
        assert o["channel_density"] * 16 == stack_gb * 1024 * 8
        cells = o["pseudochannel"] * o["sid"] * o["bankgroup"] * o["bank"] * o["row"] * o["column"]
        assert cells * o["dq"] // 8 == o["channel_density"] << 17


@pytest.mark.parametrize("org", ORGS)
def test_hbm3e_resolves_all_timings(org):
    org_dict, timing = ramulator.dram.HBM3E(org_preset=org, timing_preset="HBM3E_9600Mbps").resolve()
    assert all(v != -1 for v in timing.values())
    assert timing["tCK_ps"] == 417
    assert timing["nCCDR"] == 3
    expected_trfc = 400 if org_dict["stack_height"] == 8 else 460
    assert timing["nRFC"] == math.ceil(expected_trfc * 1000 / 417)


@pytest.mark.parametrize("org", ORGS)
def test_hbm3e_diff_sid_column_spacing_uses_nccdr(org):
    dut = make_dut(org)
    last_sid = ramulator.dram.HBM3E.org_presets[org]["sid"] - 1
    a0 = _addr(dut, sid=0, bankgroup=0, bank=0, row=0)
    a1 = _addr(dut, sid=last_sid, bankgroup=0, bank=0, row=1)
    dut.issue("ACT", a0, clk=0)
    dut.issue("ACT", a1, clk=dut.timings["nRRDS"])
    rd_clk = max(
        dut.get_first_ready_clk("RD", a0, dut.timings["nRCDRD"]),
        dut.get_first_ready_clk("RD", a1, dut.timings["nRCDRD"]),
    )
    dut.issue("RD", a0, clk=rd_clk)
    dut.assert_earliest_ready_at("RD", a1, rd_clk + dut.timings["nCCDR"])


def test_hbm3e_12hi_top_row_is_addressable():
    dut = make_dut("HBM3E_24Gb_12hi")
    a = _addr(dut, sid=2, bankgroup=3, bank=3, row=24575)
    dut.issue("ACT", a, clk=0)
    rd_clk = dut.get_first_ready_clk("RD", a, dut.timings["nRCDRD"])
    dut.issue("RD", a, clk=rd_clk)
